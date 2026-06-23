#!/home/barry/workspace/ws_moveit/venv_ocr/bin/python3
# -*- coding: utf-8 -*-

import os
import sys
import copy
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
import open3d as o3d
import rospy
import tf2_ros
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from paddleocr import PaddleOCR, draw_ocr
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as SensorImage
from visualization_msgs.msg import Marker, MarkerArray

from mv3d_rgbd_ros.srv import DetectSteelStamp, DetectSteelStampResponse


class SteelStampPrecise3DServer:
    def __init__(self):
        rospy.init_node('steel_stamp_precise_server_node')

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
        self.min_valid_points = int(rospy.get_param('~min_valid_points', 80))
        self.plane_distance_threshold = float(rospy.get_param('~plane_distance_threshold', 0.003))
        self.plane_ransac_n = int(rospy.get_param('~plane_ransac_n', 3))
        self.plane_iterations = int(rospy.get_param('~plane_iterations', 1200))
        self.normal_reference_axis = np.asarray(
            rospy.get_param('~normal_reference_axis', [0.0, 0.0, 1.0]),
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
        self.pose_z_axis_opposite_normal = bool(rospy.get_param('~pose_z_axis_opposite_normal', True))
        self.pose_axis_marker_length = float(rospy.get_param('~pose_axis_marker_length', 0.05))
        self.local_normal_radius_ratio = float(rospy.get_param('~local_normal_radius_ratio', 0.35))
        self.local_normal_min_points = int(rospy.get_param('~local_normal_min_points', 30))
        self.roi_expand_ratio = float(rospy.get_param('~roi_expand_ratio', 0.20))
        self.min_padding_px = int(rospy.get_param('~min_padding_px', 6))
        self.center_window_radius = int(rospy.get_param('~center_window_radius', 3))
        self.debug_image_prefix = rospy.get_param('~debug_image_prefix', 'precise_result')

        if np.linalg.norm(self.normal_reference_axis) < 1e-6:
            rospy.logwarn('normal_reference_axis 参数无效，回退为 base +Z')
            self.normal_reference_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        else:
            self.normal_reference_axis /= np.linalg.norm(self.normal_reference_axis)

        if np.linalg.norm(self.pose_reference_axis) < 1e-6:
            rospy.logwarn('pose_reference_axis 参数无效，回退为 base +X')
            self.pose_reference_axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        else:
            self.pose_reference_axis /= np.linalg.norm(self.pose_reference_axis)

        if np.linalg.norm(self.pose_reference_axis_backup) < 1e-6:
            rospy.logwarn('pose_reference_axis_backup 参数无效，回退为 base +Y')
            self.pose_reference_axis_backup = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        else:
            self.pose_reference_axis_backup /= np.linalg.norm(self.pose_reference_axis_backup)

        self.latest_data = {
            'rgb': None,
            'depth': None,
            'info': None,
        }
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
            det_db_thresh=0.2,
            det_db_box_thresh=0.2,
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

        rospy.loginfo('高精度 OCR 3D 节点已启动，service=%s', self.service_name)
        rospy.loginfo('订阅: rgb=%s depth=%s info=%s', self.rgb_topic, self.depth_topic, self.camera_info_topic)
        rospy.loginfo('调试发布: markers=%s poses=%s', '/debug/steel_markers', '/debug/steel_pose_array')
        rospy.loginfo('姿态配置: pose_z_axis_opposite_normal=%s normal_reference_axis=%s',
                  self.pose_z_axis_opposite_normal,
                  self.normal_reference_axis.tolist())

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

        detected_list = ocr_results[0]
        detected_list.sort(key=lambda item: np.mean(np.asarray(item[0], dtype=np.float32)[:, 0]))

        T_base_cam = self.lookup_camera_transform(rgb_msg)
        if T_base_cam is None:
            response.message = 'TF lookup failed'
            return response

        debug_image = rgb_arr.copy()
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for index, line in enumerate(detected_list):
            bbox = np.asarray(line[0], dtype=np.float32)
            text = line[1][0]
            score = float(line[1][1])

            pose_msg, center_px, debug_payload = self.compute_precise_pose(
                bbox=bbox,
                depth_img=depth_arr,
                cam_info=info_msg,
                T_base_cam=T_base_cam,
            )

            self.draw_debug_overlay(debug_image, bbox, center_px, text, score, debug_payload)

            if pose_msg is None:
                rospy.logwarn('目标 %s 高精度定位失败', text)
                continue

            response.poses.append(pose_msg)
            response.texts.append(text)
            marker_array.markers.extend(self.build_markers(index, text, pose_msg))

        self.publish_pose_array(response.poses)
        self.marker_pub.publish(marker_array)
        self.save_debug_image(debug_image)

        if response.poses:
            response.success = True
            response.message = 'Success: {} targets'.format(len(response.poses))
        else:
            response.message = 'All targets failed in precise 3D processing'

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

    def compute_precise_pose(self, bbox, depth_img, cam_info, T_base_cam):
        fx, fy = float(cam_info.P[0]), float(cam_info.P[5])
        cx, cy = float(cam_info.P[2]), float(cam_info.P[6])

        center_px = np.mean(bbox, axis=0)
        polygon, bounds = self.build_expanded_polygon(bbox, depth_img.shape)
        xmin, xmax, ymin, ymax = bounds
        if xmax <= xmin or ymax <= ymin:
            return None, center_px, {'reason': 'invalid_roi'}

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
            return None, center_px, {'reason': 'not_enough_depth', 'valid_count': valid_count}

        u_grid, v_grid = np.meshgrid(np.arange(xmin, xmax), np.arange(ymin, ymax))
        u_valid = u_grid[valid_mask].astype(np.float32)
        v_valid = v_grid[valid_mask].astype(np.float32)
        z_valid = roi_depth[valid_mask].astype(np.float32) / 1000.0

        x_valid = (u_valid - cx) * z_valid / fx
        y_valid = (v_valid - cy) * z_valid / fy
        points_cam = np.stack((x_valid, y_valid, z_valid), axis=-1)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_cam)
        if len(points_cam) < 10:
            return None, center_px, {'reason': 'pointcloud_too_small'}

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=self.plane_distance_threshold,
            ransac_n=self.plane_ransac_n,
            num_iterations=self.plane_iterations,
        )
        if len(inliers) < self.min_valid_points // 2:
            return None, center_px, {'reason': 'plane_fit_failed', 'inliers': len(inliers)}

        plane_normal_cam = np.asarray(plane_model[:3], dtype=np.float64)
        plane_normal_cam /= np.linalg.norm(plane_normal_cam)
        plane_d = float(plane_model[3])

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
            center_ray = self.pixel_to_ray(center_px[0], center_px[1], fx, fy, cx, cy)
            center_point_cam = self.intersect_ray_with_plane(center_ray, plane_normal_cam, plane_d)
            center_source = 'plane_intersection'
            center_depth_mm = None if center_point_cam is None else float(center_point_cam[2] * 1000.0)

        if center_point_cam is None:
            return None, center_px, {'reason': 'center_ray_no_intersection'}

        box_width = max(np.linalg.norm(bbox[1] - bbox[0]), np.linalg.norm(bbox[2] - bbox[3]))
        box_height = max(np.linalg.norm(bbox[3] - bbox[0]), np.linalg.norm(bbox[2] - bbox[1]))
        local_pixel_radius = max(
            self.center_window_radius + 1,
            int(round(max(box_width, box_height) * self.local_normal_radius_ratio)),
        )
        local_normal_cam, local_normal_count = self.estimate_local_surface_normal(
            center_px=center_px,
            points_cam=points_cam,
            u_valid=u_valid,
            v_valid=v_valid,
            fallback_normal_cam=plane_normal_cam,
            pixel_radius=local_pixel_radius,
        )
        if local_normal_cam is None:
            local_normal_cam = plane_normal_cam
            local_normal_count = 0

        R_base_cam = T_base_cam[:3, :3]
        t_base_cam = T_base_cam[:3, 3]

        center_point_base = R_base_cam @ center_point_cam + t_base_cam
        normal_base = R_base_cam @ local_normal_cam
        normal_base /= np.linalg.norm(normal_base)

        if np.dot(normal_base, self.normal_reference_axis) < 0.0:
            local_normal_cam = -local_normal_cam
            normal_base = -normal_base

        target_z = -normal_base if self.pose_z_axis_opposite_normal else normal_base
        target_z /= np.linalg.norm(target_z)

        target_x = self.project_reference_to_tangent_plane(target_z)
        target_y = np.cross(target_z, target_x)
        target_y /= np.linalg.norm(target_y)
        target_x = np.cross(target_y, target_z)
        target_x /= np.linalg.norm(target_x)

        rotation_matrix = np.column_stack((target_x, target_y, target_z))
        quat = R.from_matrix(rotation_matrix).as_quat()

        pose = Pose()
        pose.position.x = float(center_point_base[0])
        pose.position.y = float(center_point_base[1])
        pose.position.z = float(center_point_base[2])
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])

        debug_payload = {
            'center_depth_mm': center_depth_mm,
            'plane_depth_mm': float(center_point_cam[2] * 1000.0),
            'valid_count': valid_count,
            'inliers': len(inliers),
            'local_normal_points': int(local_normal_count),
            'local_normal_radius_px': int(local_pixel_radius),
            'center_source': center_source,
            'normal_alignment': float(np.dot(normal_base, self.normal_reference_axis)),
            'target_z_alignment': float(np.dot(target_z, self.normal_reference_axis)),
        }
        return pose, center_px, debug_payload

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

    def estimate_center_point(self, center_px, depth_img, u_valid, v_valid, points_cam, fx, fy, cx, cy):
        center_depth_mm = self.sample_center_depth(depth_img, center_px)
        if center_depth_mm is not None:
            center_point_cam = self.pixel_to_point(
                center_px[0], center_px[1], center_depth_mm / 1000.0, fx, fy, cx, cy)
            return center_point_cam, center_depth_mm, 'center_window'

        if len(points_cam) == 0:
            return None, None, 'no_valid_point'

        pixel_dist2 = np.square(u_valid - center_px[0]) + np.square(v_valid - center_px[1])
        nearest_index = int(np.argmin(pixel_dist2))
        return points_cam[nearest_index], float(points_cam[nearest_index][2] * 1000.0), 'nearest_valid'

    def estimate_local_surface_normal(self, center_px, points_cam, u_valid, v_valid, fallback_normal_cam, pixel_radius):
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

        if np.dot(normal, fallback_normal_cam) < 0.0:
            normal = -normal

        return normal, len(local_points)

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

    def intersect_ray_with_plane(self, ray, normal, d):
        denom = float(np.dot(normal, ray))
        if abs(denom) < 1e-8:
            return None
        t = -d / denom
        if t <= 0:
            return None
        return ray * t

    def draw_debug_overlay(self, image, bbox, center_px, text, score, debug_payload):
        box_int = np.round(bbox).astype(np.int32)
        cv2.polylines(image, [box_int], isClosed=True, color=(0, 255, 0), thickness=2)
        cx, cy = int(round(center_px[0])), int(round(center_px[1]))
        cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)

        label = '{} {:.2f}'.format(text, score)
        cv2.putText(
            image,
            label,
            (box_int[0][0], max(20, box_int[0][1] - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

        if 'plane_depth_mm' in debug_payload:
            info = 'plane={:.1f}mm tz={:.3f}'.format(
                debug_payload['plane_depth_mm'],
                debug_payload.get('target_z_alignment', float('nan')),
            )
            cv2.putText(
                image,
                info,
                (cx + 6, cy - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 165, 255),
                1,
                cv2.LINE_AA,
            )

    def build_markers(self, index, text, pose_msg):
        markers = []

        point_marker = Marker()
        point_marker.header.frame_id = self.base_frame
        point_marker.header.stamp = rospy.Time.now()
        point_marker.ns = 'steel_points'
        point_marker.id = index * 3
        point_marker.type = Marker.SPHERE
        point_marker.action = Marker.ADD
        point_marker.pose = copy.deepcopy(pose_msg)
        point_marker.scale.x = 0.01
        point_marker.scale.y = 0.01
        point_marker.scale.z = 0.01
        point_marker.color.r = 0.0
        point_marker.color.g = 1.0
        point_marker.color.b = 0.0
        point_marker.color.a = 1.0
        markers.append(point_marker)

        z_axis_marker = Marker()
        z_axis_marker.header.frame_id = self.base_frame
        z_axis_marker.header.stamp = rospy.Time.now()
        z_axis_marker.ns = 'steel_pose_z'
        z_axis_marker.id = index * 3 + 1
        z_axis_marker.type = Marker.ARROW
        z_axis_marker.action = Marker.ADD
        z_axis_marker.scale.x = 0.01
        z_axis_marker.scale.y = 0.015
        z_axis_marker.scale.z = 0.02
        z_axis_marker.color.b = 1.0
        z_axis_marker.color.a = 1.0
        start_point, end_point = self.build_axis_points(pose_msg, axis='z', length=self.pose_axis_marker_length)
        z_axis_marker.points = [start_point, end_point]
        markers.append(z_axis_marker)

        text_marker = Marker()
        text_marker.header.frame_id = self.base_frame
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = 'steel_text'
        text_marker.id = index * 3 + 2
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose = copy.deepcopy(pose_msg)
        text_marker.pose.position.z += 0.03
        text_marker.text = '[{}] x={:.3f} y={:.3f} z={:.3f}'.format(
            text,
            pose_msg.position.x,
            pose_msg.position.y,
            pose_msg.position.z,
        )
        text_marker.scale.z = 0.02
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        markers.append(text_marker)

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

        from geometry_msgs.msg import Point
        start_point = Point()
        start_point.x = pose_msg.position.x
        start_point.y = pose_msg.position.y
        start_point.z = pose_msg.position.z

        end_point = Point()
        end_point.x = pose_msg.position.x + float(direction[0] * length)
        end_point.y = pose_msg.position.y + float(direction[1] * length)
        end_point.z = pose_msg.position.z + float(direction[2] * length)
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
        server = SteelStampPrecise3DServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
