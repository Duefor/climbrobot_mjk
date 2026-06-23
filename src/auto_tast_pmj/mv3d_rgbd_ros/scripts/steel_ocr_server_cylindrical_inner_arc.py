#!/home/barry/workspace/ws_moveit/venv_ocr/bin/python3
# -*- coding: utf-8 -*-

import copy
import os
import sys
import threading
import time

ros_path = '/opt/ros/noetic/lib/python3/dist-packages'
sys_path = '/usr/lib/python3/dist-packages'

if os.path.exists(ros_path) and ros_path not in sys.path:
    sys.path.append(ros_path)

if os.path.exists(sys_path) and sys_path not in sys.path:
    sys.path.append(sys_path)

import cv2
import message_filters
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, PoseArray
from paddleocr import PaddleOCR
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as SensorImage
from visualization_msgs.msg import Marker, MarkerArray

from mv3d_rgbd_ros.srv import DetectSteelStamp, DetectSteelStampResponse


class SteelStampCylindricalInnerArcServer:
    """
    规则圆柱内壁单圈字符专用：
    1. 用 OCR 框 + 深度图求每个字符中心 3D 点
    2. 用字符周围局部点云估计局部法向
    3. 已知名义半径 R，通过 p_i - R * n_i 反推每个字符的圆心候选
    4. 融合所有圆心候选，得到截面圆心
    5. 将各字符点投影到名义圆上，解析得到径向外法向
    6. 令 ee_link +Z 朝向工件，即 target_z = radial_out
    7. 用圆弧切向构造 target_x，得到连续姿态
    """

    def __init__(self):
        rospy.init_node('steel_stamp_cylindrical_inner_arc_server_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.service_name = rospy.get_param('~service_name', 'get_steel_stamp_location')
        self.rgb_topic = rospy.get_param('~rgb_topic', '/camera/rgb/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/rgb/camera_info')

        self.sync_queue_size = int(rospy.get_param('~sync_queue_size', 10))
        self.sync_slop = float(rospy.get_param('~sync_slop', 0.05))
        self.depth_min_mm = int(rospy.get_param('~depth_min_mm', 200))
        self.depth_max_mm = int(rospy.get_param('~depth_max_mm', 1500))
        self.min_valid_points = int(rospy.get_param('~min_valid_points' ,80))
        ##圆弧直接设置直径
        self.nominal_diameter_m = float(rospy.get_param('~nominal_diameter_m', 2.15))
        self.nominal_radius_m = 0.5 * self.nominal_diameter_m
        self.radial_reference_axis = np.asarray(
            rospy.get_param('~radial_reference_axis', [0.0, 0.0, 1.0]),
            dtype=np.float64,
        )
        self.pose_reference_axis = np.asarray(
            rospy.get_param('~pose_reference_axis', [1.0, 0.0, 0.0]),
            dtype=np.float64,
        )
        self.pose_reference_axis_backup = np.asarray(
            rospy.get_param('~pose_reference_axis_backup', [0.0, 1.0, 0.0]),
            dtype=np.float64,
        )

        self.local_normal_radius_ratio = float(rospy.get_param('~local_normal_radius_ratio', 0.35))
        self.local_normal_min_points = int(rospy.get_param('~local_normal_min_points', 30))
        self.center_window_radius = int(rospy.get_param('~center_window_radius', 3))
        self.roi_expand_ratio = float(rospy.get_param('~roi_expand_ratio', 0.20))
        self.min_padding_px = int(rospy.get_param('~min_padding_px', 6))
        self.center_candidate_outlier_m = float(rospy.get_param('~center_candidate_outlier_m', 0.08))
        self.circle_plane_min_points = int(rospy.get_param('~circle_plane_min_points', 3))
        self.arc_tangent_length = float(rospy.get_param('~arc_tangent_length', 0.05))
        self.trajectory_mode = rospy.get_param('~trajectory_mode', 'dense_arc')
        self.arc_sample_step_m = float(rospy.get_param('~arc_sample_step_m', 0.004))
        self.max_arc_points = int(rospy.get_param('~max_arc_points', 500))
        self.pose_z_to_center = bool(rospy.get_param('~pose_z_to_center', True))
        self.debug_image_prefix = rospy.get_param('~debug_image_prefix', 'cyl_inner_arc_result')
        self.sort_by = rospy.get_param('~sort_by', 'ocr_x')
        # self.sort_by = rospy.get_param('~sort_by', 'arc_angle')
        
        if np.linalg.norm(self.radial_reference_axis) < 1e-6:
            self.radial_reference_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        else:
            self.radial_reference_axis /= np.linalg.norm(self.radial_reference_axis)

        if np.linalg.norm(self.pose_reference_axis) < 1e-6:
            self.pose_reference_axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        else:
            self.pose_reference_axis /= np.linalg.norm(self.pose_reference_axis)

        if np.linalg.norm(self.pose_reference_axis_backup) < 1e-6:
            self.pose_reference_axis_backup = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        else:
            self.pose_reference_axis_backup /= np.linalg.norm(self.pose_reference_axis_backup)

        self.latest_data = {'rgb': None, 'depth': None, 'info': None}
        self.data_lock = threading.Lock()

        self.det_model_path = '/home/barry/workspace/ws_moveit/PaddleOCR/inference/det_steel_1280x720/'
        self.rec_model_path = '/home/barry/workspace/ws_moveit/PaddleOCR/inference/rec_single_char/'
        self.ocr = PaddleOCR(
            use_angle_cls=True,
            lang='en',
            use_gpu=True,
            det_model_dir=self.det_model_path,
            rec_model_dir=self.rec_model_path,
            rec=True,
            det_algorithm='DB',
            det_db_thresh=0.5,
            det_db_box_thresh=0.5,
            det_db_unclip_ratio=1.6,
            show_log=False,
        )

        script_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_root = os.path.dirname(script_dir)
        self.save_dir = os.path.join(pkg_root, 'debug_images')
        os.makedirs(self.save_dir, exist_ok=True)

        rgb_sub = message_filters.Subscriber(self.rgb_topic, SensorImage)
        depth_sub = message_filters.Subscriber(self.depth_topic, SensorImage)
        info_sub = message_filters.Subscriber(self.camera_info_topic, CameraInfo)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub],
            self.sync_queue_size,
            self.sync_slop,
        )
        self.sync.registerCallback(self.sync_callback)

        self.srv = rospy.Service(self.service_name, DetectSteelStamp, self.handle_req)
        self.marker_pub = rospy.Publisher('/debug/steel_markers', MarkerArray, queue_size=10, latch=True)
        self.pose_array_pub = rospy.Publisher('/debug/steel_pose_array', PoseArray, queue_size=1, latch=True)

        rospy.loginfo('圆柱内壁 OCR 节点已启动, service=%s', self.service_name)
        rospy.loginfo('名义直径=%.4f m 半径=%.4f m', self.nominal_diameter_m, self.nominal_radius_m)
        rospy.loginfo('radial_reference_axis=%s', self.radial_reference_axis.tolist())
        rospy.loginfo('trajectory_mode=%s arc_sample_step_m=%.4f pose_z_to_center=%s',
                  self.trajectory_mode, self.arc_sample_step_m, str(self.pose_z_to_center))

    def sync_callback(self, rgb_msg, depth_msg, info_msg):
        with self.data_lock:
            self.latest_data['rgb'] = rgb_msg
            self.latest_data['depth'] = depth_msg
            self.latest_data['info'] = info_msg

    def handle_req(self, _req):
        response = DetectSteelStampResponse()
        response.success = False

        with self.data_lock:
            rgb_msg = self.latest_data['rgb']
            depth_msg = self.latest_data['depth']
            info_msg = self.latest_data['info']

        if rgb_msg is None or depth_msg is None or info_msg is None:
            response.message = 'No synchronized RGB/Depth/CameraInfo received yet'
            rospy.logwarn(response.message)
            return response

        try:
            rgb_arr = self.decode_bgr8(rgb_msg)
            depth_arr = self.decode_depth16(depth_msg)
        except Exception as exc:
            response.message = 'Image parse error: {}'.format(exc)
            rospy.logerr(response.message)
            return response

        ocr_results = self.ocr.ocr(rgb_arr, cls=False)
        if not ocr_results or not ocr_results[0]:
            response.message = 'OCR detected nothing'
            return response

        detections = ocr_results[0]
        detections.sort(key=lambda item: np.mean(np.asarray(item[0], dtype=np.float32)[:, 0]))

        T_base_cam = self.lookup_camera_transform(rgb_msg)
        if T_base_cam is None:
            response.message = 'TF lookup failed'
            return response

        debug_image = rgb_arr.copy()
        valid_entries = []

        for line in detections:
            bbox = np.asarray(line[0], dtype=np.float32)
            text = line[1][0]
            score = float(line[1][1])

            result = self.compute_character_geometry(
                bbox=bbox,
                depth_img=depth_arr,
                cam_info=info_msg,
                T_base_cam=T_base_cam,
            )

            self.draw_debug_overlay(debug_image, bbox, result['center_px'], text, score, result['debug'])

            if not result['success']:
                rospy.logwarn('目标 %s 几何求解失败: %s', text, result['debug'].get('reason', 'unknown'))
                continue

            entry = {
                'text': text,
                'score': score,
                'bbox': bbox,
                'center_px': result['center_px'],
                'point_base': result['point_base'],
                'normal_base': result['normal_base'],
                'debug': result['debug'],
            }
            valid_entries.append(entry)

        if not valid_entries:
            response.message = 'No valid characters after 3D processing'
            self.save_debug_image(debug_image)
            return response

        if self.sort_by == 'arc_angle':
            valid_entries = self.sort_entries_by_arc_hint(valid_entries)

        circle_model = self.estimate_circle_model(valid_entries)
        if circle_model is None:
            response.message = 'Failed to estimate circle model'
            self.save_debug_image(debug_image)
            return response

        valid_entries = self.build_arc_poses(valid_entries, circle_model)
        if self.trajectory_mode == 'dense_arc':
            dense_poses = self.densify_arc_poses(valid_entries, circle_model)
            if dense_poses:
                response.poses.extend(dense_poses)
                response.texts = ['ARC'] * len(dense_poses)
            else:
                response.message = 'Dense arc generation failed'
                self.save_debug_image(debug_image)
                return response

        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        marker_array.markers.extend(self.build_circle_center_markers(circle_model))

        if self.trajectory_mode == 'dense_arc':
            for index, entry in enumerate(valid_entries):
                marker_array.markers.extend(self.build_markers(
                    index=index,
                    text=entry['text'],
                    pose_msg=entry['pose'],
                    ns_prefix='char_pose',
                    id_offset=10000,
                    point_scale=0.018,
                    axis_length=max(self.arc_tangent_length, 0.05),
                    text_scale=0.028,
                    text_offset=0.04,
                    show_axis=True,
                    show_text=True,
                ))

            for index, pose_msg in enumerate(response.poses):
                marker_array.markers.extend(self.build_markers(
                    index=index,
                    text='ARC_{:03d}'.format(index),
                    pose_msg=pose_msg,
                    ns_prefix='arc_dense',
                    id_offset=0,
                    point_scale=0.006,
                    axis_length=max(self.arc_tangent_length * 0.5, 0.01),
                    text_scale=0.015,
                    text_offset=0.02,
                    show_axis=False,
                    show_text=False,
                ))
        else:
            for index, entry in enumerate(valid_entries):
                pose_msg = entry['pose']
                response.poses.append(pose_msg)
                response.texts.append(entry['text'])
                marker_array.markers.extend(self.build_markers(index, entry['text'], pose_msg))

        self.publish_pose_array(response.poses)
        self.marker_pub.publish(marker_array)
        self.save_debug_image(debug_image)

        response.success = len(response.poses) > 0
        if response.success:
            response.message = 'Success: {} targets'.format(len(response.poses))
        else:
            response.message = 'No valid poses after circle fitting'
        return response

    def decode_bgr8(self, msg):
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        row_stride = int(msg.step)
        expected = int(msg.width) * 3
        image = raw.reshape(int(msg.height), row_stride)[:, :expected]
        return image.reshape(int(msg.height), int(msg.width), 3).copy()

    def decode_depth16(self, msg):
        raw = np.frombuffer(msg.data, dtype=np.uint16)
        row_stride = int(msg.step) // 2
        image = raw.reshape(int(msg.height), row_stride)[:, :int(msg.width)]
        return image.copy()

    def lookup_camera_transform(self, rgb_msg):
        stamp = rgb_msg.header.stamp
        camera_frame = rgb_msg.header.frame_id
        try:
            trans_stamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                camera_frame,
                stamp,
                rospy.Duration(1.0),
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            try:
                trans_stamped = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    camera_frame,
                    rospy.Time(0),
                    rospy.Duration(1.0),
                )
                rospy.logwarn_throttle(5.0, 'TF 使用最新变换作为回退，建议检查时间同步')
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as exc:
                rospy.logerr('TF Error: %s', exc)
                return None
        return self.ros_transform_to_matrix(trans_stamped.transform)

    def compute_character_geometry(self, bbox, depth_img, cam_info, T_base_cam):
        fx, fy = float(cam_info.P[0]), float(cam_info.P[5])
        cx, cy = float(cam_info.P[2]), float(cam_info.P[6])

        center_px = np.mean(bbox, axis=0)
        polygon, bounds = self.build_expanded_polygon(bbox, depth_img.shape)
        xmin, xmax, ymin, ymax = bounds
        if xmax <= xmin or ymax <= ymin:
            return {'success': False, 'center_px': center_px, 'debug': {'reason': 'invalid_roi'}}

        roi_depth = depth_img[ymin:ymax, xmin:xmax]
        polygon_roi = polygon - np.array([xmin, ymin], dtype=np.float32)
        polygon_mask = np.zeros((ymax - ymin, xmax - xmin), dtype=np.uint8)
        cv2.fillConvexPoly(polygon_mask, np.round(polygon_roi).astype(np.int32), 255)

        valid_mask = (
            (polygon_mask > 0)
            & (roi_depth >= self.depth_min_mm)
            & (roi_depth <= self.depth_max_mm)
        )
        valid_count = int(np.count_nonzero(valid_mask))
        if valid_count < self.min_valid_points:
            return {'success': False, 'center_px': center_px, 'debug': {'reason': 'not_enough_depth', 'valid_count': valid_count}}

        u_grid, v_grid = np.meshgrid(np.arange(xmin, xmax), np.arange(ymin, ymax))
        u_valid = u_grid[valid_mask].astype(np.float32)
        v_valid = v_grid[valid_mask].astype(np.float32)
        z_valid = roi_depth[valid_mask].astype(np.float32) / 1000.0
        x_valid = (u_valid - cx) * z_valid / fx
        y_valid = (v_valid - cy) * z_valid / fy
        points_cam = np.stack((x_valid, y_valid, z_valid), axis=-1)

        if len(points_cam) < 10:
            return {'success': False, 'center_px': center_px, 'debug': {'reason': 'pointcloud_too_small'}}

        center_point_cam, center_depth_mm, center_source = self.estimate_center_point(
            center_px=center_px,
            depth_img=depth_img,
            u_valid=u_valid,
            v_valid=v_valid,
            points_cam=points_cam,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
        )
        if center_point_cam is None:
            return {'success': False, 'center_px': center_px, 'debug': {'reason': 'center_point_failed'}}

        local_pixel_radius = max(
            self.center_window_radius + 1,
            int(round(max(np.linalg.norm(bbox[1] - bbox[0]), np.linalg.norm(bbox[2] - bbox[3])) * self.local_normal_radius_ratio)),
        )

        local_normal_cam, local_normal_count = self.estimate_local_surface_normal(
            center_px=center_px,
            points_cam=points_cam,
            u_valid=u_valid,
            v_valid=v_valid,
            pixel_radius=local_pixel_radius,
        )
        if local_normal_cam is None:
            return {'success': False, 'center_px': center_px, 'debug': {'reason': 'local_normal_failed'}}

        R_base_cam = T_base_cam[:3, :3]
        t_base_cam = T_base_cam[:3, 3]
        point_base = R_base_cam @ center_point_cam + t_base_cam
        normal_base = R_base_cam @ local_normal_cam
        normal_base /= np.linalg.norm(normal_base)

        if np.dot(normal_base, self.radial_reference_axis) < 0.0:
            normal_base = -normal_base

        center_candidate = point_base - self.nominal_radius_m * normal_base
        debug = {
            'center_depth_mm': center_depth_mm,
            'center_source': center_source,
            'valid_count': valid_count,
            'local_normal_points': int(local_normal_count),
            'center_candidate': center_candidate.tolist(),
            'normal_alignment': float(np.dot(normal_base, self.radial_reference_axis)),
        }

        return {
            'success': True,
            'center_px': center_px,
            'point_base': point_base,
            'normal_base': normal_base,
            'debug': debug,
        }

    def estimate_center_point(self, center_px, depth_img, u_valid, v_valid, points_cam, fx, fy, cx, cy):
        center_depth_mm = self.sample_center_depth(depth_img, center_px)
        if center_depth_mm is not None:
            center_point_cam = self.pixel_to_point(center_px[0], center_px[1], center_depth_mm / 1000.0, fx, fy, cx, cy)
            return center_point_cam, center_depth_mm, 'center_window'

        if len(points_cam) == 0:
            return None, None, 'no_valid_point'

        pixel_dist2 = np.square(u_valid - center_px[0]) + np.square(v_valid - center_px[1])
        nearest_index = int(np.argmin(pixel_dist2))
        return points_cam[nearest_index], float(points_cam[nearest_index][2] * 1000.0), 'nearest_valid'

    def estimate_local_surface_normal(self, center_px, points_cam, u_valid, v_valid, pixel_radius):
        pixel_dist2 = np.square(u_valid - center_px[0]) + np.square(v_valid - center_px[1])
        neighbor_mask = pixel_dist2 <= float(pixel_radius * pixel_radius)
        local_points = points_cam[neighbor_mask]

        if len(local_points) < self.local_normal_min_points:
            nearest_count = min(max(self.local_normal_min_points, 3), len(points_cam))
            nearest_indices = np.argsort(pixel_dist2)[:nearest_count]
            local_points = points_cam[nearest_indices]

        if len(local_points) < 3:
            return None, 0

        centered = local_points - np.mean(local_points, axis=0, keepdims=True)
        covariance = centered.T @ centered / max(len(local_points) - 1, 1)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        normal = eigenvectors[:, np.argmin(eigenvalues)]
        normal /= np.linalg.norm(normal)
        return normal, len(local_points)

    def estimate_circle_model(self, entries):
        points = np.asarray([entry['point_base'] for entry in entries], dtype=np.float64)
        normals = np.asarray([entry['normal_base'] for entry in entries], dtype=np.float64)
        center_candidates = points - self.nominal_radius_m * normals

        center_seed = np.median(center_candidates, axis=0)
        distances = np.linalg.norm(center_candidates - center_seed[None, :], axis=1)
        inlier_mask = distances <= self.center_candidate_outlier_m
        if np.count_nonzero(inlier_mask) == 0:
            inlier_mask = np.ones(len(entries), dtype=bool)
        center_fused = np.mean(center_candidates[inlier_mask], axis=0)

        if len(points) >= 3:
            plane_origin = np.mean(points, axis=0)
            centered_points = points - plane_origin[None, :]
            covariance = centered_points.T @ centered_points / max(len(points) - 1, 1)
            eigenvalues, eigenvectors = np.linalg.eigh(covariance)
            plane_normal = eigenvectors[:, np.argmin(eigenvalues)]
            plane_normal /= np.linalg.norm(plane_normal)
        elif len(points) == 2:
            radial0 = points[0] - center_fused
            radial1 = points[1] - center_fused
            plane_normal = np.cross(radial0, radial1)
            if np.linalg.norm(plane_normal) < 1e-6:
                plane_normal = np.cross(self.radial_reference_axis, self.pose_reference_axis)
            plane_normal /= np.linalg.norm(plane_normal)
            plane_origin = np.mean(points, axis=0)
        else:
            plane_normal = np.cross(self.radial_reference_axis, self.pose_reference_axis)
            if np.linalg.norm(plane_normal) < 1e-6:
                plane_normal = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            plane_normal /= np.linalg.norm(plane_normal)
            plane_origin = points[0]

        center_on_plane = center_fused - np.dot(center_fused - plane_origin, plane_normal) * plane_normal

        rospy.loginfo('圆弧模型: center=(%.4f, %.4f, %.4f) plane_normal=(%.4f, %.4f, %.4f) inliers=%d/%d',
                      center_on_plane[0], center_on_plane[1], center_on_plane[2],
                      plane_normal[0], plane_normal[1], plane_normal[2],
                      int(np.count_nonzero(inlier_mask)), len(entries))

        return {
            'center': center_on_plane,
            'plane_normal': plane_normal,
            'plane_origin': plane_origin,
            'radius': self.nominal_radius_m,
            'center_candidates': center_candidates,
            'inlier_mask': inlier_mask,
        }

    def build_arc_poses(self, entries, circle_model):
        center = circle_model['center']
        plane_normal = circle_model['plane_normal']
        plane_origin = circle_model['plane_origin']
        radius = circle_model['radius']

        projected_points = []
        for entry in entries:
            point = entry['point_base']
            point_on_plane = point - np.dot(point - plane_origin, plane_normal) * plane_normal
            radial = point_on_plane - center
            radial_norm = np.linalg.norm(radial)
            if radial_norm < 1e-6:
                radial = entry['normal_base']
                radial_norm = np.linalg.norm(radial)
            radial /= radial_norm
            point_on_circle = center + radius * radial
            entry['projected_point_base'] = point_on_circle
            entry['radial_out_base'] = radial
            projected_points.append(point_on_circle)

        projected_points = np.asarray(projected_points, dtype=np.float64)
        previous_tangent = None

        for index, entry in enumerate(entries):
            target_z = self.get_target_z_from_radial(entry['radial_out_base'])
            tangent = self.estimate_arc_tangent(index, projected_points)
            tangent = tangent - np.dot(tangent, target_z) * target_z
            tangent_norm = np.linalg.norm(tangent)
            if tangent_norm < 1e-6:
                tangent = self.project_reference_to_tangent_plane(target_z)
            else:
                tangent /= tangent_norm

            if previous_tangent is not None and np.dot(tangent, previous_tangent) < 0.0:
                tangent = -tangent
            previous_tangent = tangent

            target_x = tangent
            target_y = np.cross(target_z, target_x)
            target_y /= np.linalg.norm(target_y)
            target_x = np.cross(target_y, target_z)
            target_x /= np.linalg.norm(target_x)

            rotation_matrix = np.column_stack((target_x, target_y, target_z))
            quat = R.from_matrix(rotation_matrix).as_quat()

            pose = Pose()
            pose.position.x = float(entry['projected_point_base'][0])
            pose.position.y = float(entry['projected_point_base'][1])
            pose.position.z = float(entry['projected_point_base'][2])
            pose.orientation.x = float(quat[0])
            pose.orientation.y = float(quat[1])
            pose.orientation.z = float(quat[2])
            pose.orientation.w = float(quat[3])
            entry['pose'] = pose
            entry['debug']['target_z_alignment'] = float(np.dot(target_z, self.radial_reference_axis))
            entry['debug']['circle_center'] = center.tolist()
            entry['debug']['projected_point_base'] = entry['projected_point_base'].tolist()

        return entries

    def densify_arc_poses(self, entries, circle_model):
        if len(entries) == 0:
            return []
        if len(entries) == 1:
            return [copy.deepcopy(entries[0]['pose'])]

        center = circle_model['center']
        plane_normal = circle_model['plane_normal']
        radius = float(circle_model['radius'])
        if radius <= 1e-6:
            return []

        radial0 = entries[0]['radial_out_base']
        radial0 = radial0 / np.linalg.norm(radial0)
        basis_u = radial0
        basis_v = np.cross(plane_normal, basis_u)
        basis_v_norm = np.linalg.norm(basis_v)
        if basis_v_norm < 1e-6:
            basis_u = self.project_reference_to_tangent_plane(plane_normal)
            basis_v = np.cross(plane_normal, basis_u)
            basis_v_norm = np.linalg.norm(basis_v)
        basis_v /= max(basis_v_norm, 1e-6)

        raw_angles = []
        for entry in entries:
            radial = entry['radial_out_base']
            angle = float(np.arctan2(np.dot(radial, basis_v), np.dot(radial, basis_u)))
            raw_angles.append(angle)

        unwrapped = [raw_angles[0]]
        for angle in raw_angles[1:]:
            candidate = angle
            while candidate - unwrapped[-1] > np.pi:
                candidate -= 2.0 * np.pi
            while candidate - unwrapped[-1] < -np.pi:
                candidate += 2.0 * np.pi
            unwrapped.append(candidate)

        step = max(float(self.arc_sample_step_m), 1e-4)
        theta_step = step / radius
        dense_angles = [unwrapped[0]]
        for start_theta, end_theta in zip(unwrapped[:-1], unwrapped[1:]):
            delta = end_theta - start_theta
            if abs(delta) < 1e-9:
                continue
            count = int(np.ceil(abs(delta) / theta_step))
            count = max(1, count)
            for i in range(1, count + 1):
                dense_angles.append(start_theta + delta * (float(i) / float(count)))

        if len(dense_angles) > self.max_arc_points:
            sample_index = np.linspace(0, len(dense_angles) - 1, self.max_arc_points).astype(np.int32)
            dense_angles = [dense_angles[idx] for idx in sample_index]

        overall_direction = 1.0 if (unwrapped[-1] - unwrapped[0]) >= 0.0 else -1.0
        previous_tangent = None
        previous_quat = None
        dense_poses = []

        for theta in dense_angles:
            radial = np.cos(theta) * basis_u + np.sin(theta) * basis_v
            radial /= np.linalg.norm(radial)
            point = center + radius * radial

            target_z = self.get_target_z_from_radial(radial)
            tangent = overall_direction * (-np.sin(theta) * basis_u + np.cos(theta) * basis_v)
            tangent = tangent - np.dot(tangent, target_z) * target_z
            tangent_norm = np.linalg.norm(tangent)
            if tangent_norm < 1e-6:
                tangent = self.project_reference_to_tangent_plane(target_z)
            else:
                tangent /= tangent_norm

            if previous_tangent is not None and np.dot(tangent, previous_tangent) < 0.0:
                tangent = -tangent
            previous_tangent = tangent

            target_x = tangent
            target_y = np.cross(target_z, target_x)
            target_y /= np.linalg.norm(target_y)
            target_x = np.cross(target_y, target_z)
            target_x /= np.linalg.norm(target_x)

            rotation_matrix = np.column_stack((target_x, target_y, target_z))
            quat = R.from_matrix(rotation_matrix).as_quat()
            if previous_quat is not None and np.dot(quat, previous_quat) < 0.0:
                quat = -quat
            previous_quat = quat

            pose = Pose()
            pose.position.x = float(point[0])
            pose.position.y = float(point[1])
            pose.position.z = float(point[2])
            pose.orientation.x = float(quat[0])
            pose.orientation.y = float(quat[1])
            pose.orientation.z = float(quat[2])
            pose.orientation.w = float(quat[3])
            dense_poses.append(pose)

        rospy.loginfo('Dense arc generated: key=%d dense=%d step=%.4fm',
                      len(entries), len(dense_poses), step)
        return dense_poses

    def get_target_z_from_radial(self, radial_out):
        if self.pose_z_to_center:
            return -radial_out
        return radial_out

    def estimate_arc_tangent(self, index, projected_points):
        if len(projected_points) <= 1:
            return self.pose_reference_axis.copy()
        if index == 0:
            tangent = projected_points[1] - projected_points[0]
        elif index == len(projected_points) - 1:
            tangent = projected_points[-1] - projected_points[-2]
        else:
            tangent = projected_points[index + 1] - projected_points[index - 1]
        tangent_norm = np.linalg.norm(tangent)
        if tangent_norm < 1e-6:
            return self.pose_reference_axis.copy()
        return tangent / tangent_norm

    def sort_entries_by_arc_hint(self, entries):
        points = np.asarray([entry['point_base'] for entry in entries], dtype=np.float64)
        plane_origin = np.mean(points, axis=0)
        centered_points = points - plane_origin[None, :]
        covariance = centered_points.T @ centered_points / max(len(points) - 1, 1)
        _, eigenvectors = np.linalg.eigh(covariance)
        basis_u = eigenvectors[:, -1]
        basis_u -= np.dot(basis_u, self.radial_reference_axis) * self.radial_reference_axis
        if np.linalg.norm(basis_u) < 1e-6:
            basis_u = self.project_reference_to_tangent_plane(self.radial_reference_axis)
        else:
            basis_u /= np.linalg.norm(basis_u)
        basis_v = np.cross(self.radial_reference_axis, basis_u)
        basis_v /= np.linalg.norm(basis_v)

        for entry in entries:
            point = entry['point_base'] - plane_origin
            entry['arc_angle_hint'] = float(np.arctan2(np.dot(point, basis_v), np.dot(point, basis_u)))
        return sorted(entries, key=lambda item: item['arc_angle_hint'])

    def build_expanded_polygon(self, bbox, image_shape):
        center = np.mean(bbox, axis=0)
        expanded = center + (bbox - center) * (1.0 + self.roi_expand_ratio)
        min_xy = np.min(bbox, axis=0)
        max_xy = np.max(bbox, axis=0)
        expanded[:, 0] = np.clip(expanded[:, 0], 0, image_shape[1] - 1)
        expanded[:, 1] = np.clip(expanded[:, 1], 0, image_shape[0] - 1)
        xmin = max(0, int(np.floor(min(np.min(expanded[:, 0]), min_xy[0]) - self.min_padding_px)))
        xmax = min(image_shape[1], int(np.ceil(max(np.max(expanded[:, 0]), max_xy[0]) + self.min_padding_px)))
        ymin = max(0, int(np.floor(min(np.min(expanded[:, 1]), min_xy[1]) - self.min_padding_px)))
        ymax = min(image_shape[0], int(np.ceil(max(np.max(expanded[:, 1]), max_xy[1]) + self.min_padding_px)))
        return expanded.astype(np.float32), (xmin, xmax, ymin, ymax)

    def sample_center_depth(self, depth_img, center_px):
        u = int(round(center_px[0]))
        v = int(round(center_px[1]))
        radius = max(1, self.center_window_radius)
        xmin = max(0, u - radius)
        xmax = min(depth_img.shape[1], u + radius + 1)
        ymin = max(0, v - radius)
        ymax = min(depth_img.shape[0], v + radius + 1)
        window = depth_img[ymin:ymax, xmin:xmax]
        valid = window[(window >= self.depth_min_mm) & (window <= self.depth_max_mm)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def pixel_to_point(self, u, v, z, fx, fy, cx, cy):
        return np.array([
            (float(u) - cx) * z / fx,
            (float(v) - cy) * z / fy,
            float(z),
        ], dtype=np.float64)

    def pixel_to_ray(self, u, v, fx, fy, cx, cy):
        ray = np.array([
            (float(u) - cx) / fx,
            (float(v) - cy) / fy,
            1.0,
        ], dtype=np.float64)
        return ray / np.linalg.norm(ray)

    def project_reference_to_tangent_plane(self, target_z):
        for reference_axis in (
            self.pose_reference_axis,
            self.pose_reference_axis_backup,
            np.array([0.0, 0.0, 1.0], dtype=np.float64),
        ):
            tangent = reference_axis - np.dot(reference_axis, target_z) * target_z
            tangent_norm = np.linalg.norm(tangent)
            if tangent_norm > 1e-6:
                return tangent / tangent_norm
        raise ValueError('Failed to construct tangent reference axis from configured pose axes')

    def draw_debug_overlay(self, image, bbox, center_px, text, score, debug_payload):
        box_int = np.round(bbox).astype(np.int32)
        cv2.polylines(image, [box_int], isClosed=True, color=(0, 255, 0), thickness=2)
        cx, cy = int(round(center_px[0])), int(round(center_px[1]))
        cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)
        label = '{} {:.2f}'.format(text, score)
        cv2.putText(image, label, (box_int[0][0], max(20, box_int[0][1] - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)
        if 'center_depth_mm' in debug_payload:
            info = 'z={:.1f}mm na={:.3f}'.format(
                debug_payload.get('center_depth_mm', 0.0),
                debug_payload.get('normal_alignment', float('nan')),
            )
            cv2.putText(image, info, (cx + 6, cy - 6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 165, 255), 1, cv2.LINE_AA)

    def build_markers(self,
                      index,
                      text,
                      pose_msg,
                      ns_prefix='steel',
                      id_offset=0,
                      point_scale=0.01,
                      axis_length=None,
                      text_scale=0.02,
                      text_offset=0.03,
                      show_axis=True,
                      show_text=True):
        markers = []
        axis_length = self.arc_tangent_length if axis_length is None else axis_length

        point_marker = Marker()
        point_marker.header.frame_id = self.base_frame
        point_marker.header.stamp = rospy.Time.now()
        point_marker.ns = '{}_points'.format(ns_prefix)
        point_marker.id = id_offset + index * 3
        point_marker.type = Marker.SPHERE
        point_marker.action = Marker.ADD
        point_marker.pose = copy.deepcopy(pose_msg)
        point_marker.scale.x = point_scale
        point_marker.scale.y = point_scale
        point_marker.scale.z = point_scale
        point_marker.color.g = 1.0
        point_marker.color.a = 1.0
        markers.append(point_marker)

        if show_axis:
            z_axis_marker = Marker()
            z_axis_marker.header.frame_id = self.base_frame
            z_axis_marker.header.stamp = rospy.Time.now()
            z_axis_marker.ns = '{}_pose_z'.format(ns_prefix)
            z_axis_marker.id = id_offset + index * 3 + 1
            z_axis_marker.type = Marker.ARROW
            z_axis_marker.action = Marker.ADD
            z_axis_marker.scale.x = 0.01
            z_axis_marker.scale.y = 0.015
            z_axis_marker.scale.z = 0.02
            z_axis_marker.color.b = 1.0
            z_axis_marker.color.a = 1.0
            start_point, end_point = self.build_axis_points(pose_msg, axis='z', length=axis_length)
            z_axis_marker.points = [start_point, end_point]
            markers.append(z_axis_marker)

        if show_text:
            text_marker = Marker()
            text_marker.header.frame_id = self.base_frame
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = '{}_text'.format(ns_prefix)
            text_marker.id = id_offset + index * 3 + 2
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = copy.deepcopy(pose_msg)
            text_marker.pose.position.z += text_offset
            text_marker.text = '[{}] x={:.3f} y={:.3f} z={:.3f}'.format(
                text,
                pose_msg.position.x,
                pose_msg.position.y,
                pose_msg.position.z,
            )
            text_marker.scale.z = text_scale
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            markers.append(text_marker)
        return markers

    def build_circle_center_markers(self, circle_model):
        markers = []
        center = circle_model['center']
        plane_normal = circle_model['plane_normal']

        center_marker = Marker()
        center_marker.header.frame_id = self.base_frame
        center_marker.header.stamp = rospy.Time.now()
        center_marker.ns = 'circle_center'
        center_marker.id = 9000
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.pose.orientation.w = 1.0
        center_marker.pose.position.x = float(center[0])
        center_marker.pose.position.y = float(center[1])
        center_marker.pose.position.z = float(center[2])
        center_marker.scale.x = 0.03
        center_marker.scale.y = 0.03
        center_marker.scale.z = 0.03
        center_marker.color.r = 1.0
        center_marker.color.g = 1.0
        center_marker.color.a = 1.0
        markers.append(center_marker)

        axis_marker = Marker()
        axis_marker.header.frame_id = self.base_frame
        axis_marker.header.stamp = rospy.Time.now()
        axis_marker.ns = 'circle_plane_normal'
        axis_marker.id = 9001
        axis_marker.type = Marker.ARROW
        axis_marker.action = Marker.ADD
        axis_marker.scale.x = 0.01
        axis_marker.scale.y = 0.02
        axis_marker.scale.z = 0.02
        axis_marker.color.r = 1.0
        axis_marker.color.a = 1.0
        start_point = Point(x=float(center[0]), y=float(center[1]), z=float(center[2]))
        end_point = Point(
            x=float(center[0] + plane_normal[0] * 0.1),
            y=float(center[1] + plane_normal[1] * 0.1),
            z=float(center[2] + plane_normal[2] * 0.1),
        )
        axis_marker.points = [start_point, end_point]
        markers.append(axis_marker)
        return markers

    def build_axis_points(self, pose_msg, axis='z', length=0.05):
        rotation = R.from_quat([
            pose_msg.orientation.x,
            pose_msg.orientation.y,
            pose_msg.orientation.z,
            pose_msg.orientation.w,
        ])
        axis_map = {
            'x': np.array([1.0, 0.0, 0.0], dtype=np.float64),
            'y': np.array([0.0, 1.0, 0.0], dtype=np.float64),
            'z': np.array([0.0, 0.0, 1.0], dtype=np.float64),
        }
        direction = rotation.apply(axis_map[axis])
        start_point = Point(x=pose_msg.position.x, y=pose_msg.position.y, z=pose_msg.position.z)
        end_point = Point(
            x=pose_msg.position.x + float(direction[0] * length),
            y=pose_msg.position.y + float(direction[1] * length),
            z=pose_msg.position.z + float(direction[2] * length),
        )
        return start_point, end_point

    def publish_pose_array(self, poses):
        pose_array = PoseArray()
        pose_array.header.frame_id = self.base_frame
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses = [copy.deepcopy(pose) for pose in poses]
        self.pose_array_pub.publish(pose_array)

    def save_debug_image(self, debug_image):
        filename = '{}_{}.jpg'.format(self.debug_image_prefix, int(time.time() * 1000))
        cv2.imwrite(os.path.join(self.save_dir, filename), debug_image)

    def ros_transform_to_matrix(self, trans):
        q = trans.rotation
        t = trans.translation
        mat = np.eye(4)
        mat[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        mat[:3, 3] = [t.x, t.y, t.z]
        return mat


if __name__ == '__main__':
    try:
        server = SteelStampCylindricalInnerArcServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
