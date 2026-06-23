#!/usr/bin/env python3
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
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as SensorImage
from visualization_msgs.msg import Marker, MarkerArray

from mv3d_rgbd_ros.srv import DetectSteelStamp, DetectSteelStampResponse


class SteelStampCylindricalGlobalSurfaceServer:
    """
    全局曲面版钢印 OCR 节点（替代逐字符局部深度估计）：
    1) OCR 识别字符框
    2) 合并字符框得到全局 ROI，提取宏观点云
    3) 对全局点云做 SOR 去飞点
    4) 拟合圆柱曲面（轴线 + 半径）
    5) 对每个字符中心像素做“射线-圆柱求交”，得到稳定 3D 点与法向
    6) 基于字符 3D 点生成平滑圆弧轨迹与姿态
    """

    def __init__(self):
        rospy.init_node('steel_stamp_cylindrical_global_surface_server_node')

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
        self.depth_max_mm = int(rospy.get_param('~depth_max_mm', 700))
        self.min_valid_points = int(rospy.get_param('~min_valid_points', 200))

        self.nominal_diameter_m = float(rospy.get_param('~nominal_diameter_m', 4.3))
        self.nominal_radius_m = 0.5 * self.nominal_diameter_m
        self.radius_search_min_m = float(rospy.get_param('~radius_search_min_m', self.nominal_radius_m - 0.03))
        self.radius_search_max_m = float(rospy.get_param('~radius_search_max_m', self.nominal_radius_m + 0.03))

        self.radial_reference_axis = np.asarray(
            rospy.get_param('~radial_reference_axis', [0.0, 0.0, 1.0]), dtype=np.float64
        )
        self.pose_reference_axis = np.asarray(
            rospy.get_param('~pose_reference_axis', [1.0, 0.0, 0.0]), dtype=np.float64
        )
        self.pose_reference_axis_backup = np.asarray(
            rospy.get_param('~pose_reference_axis_backup', [0.0, 1.0, 0.0]), dtype=np.float64
        )

        self.global_roi_expand_px = int(rospy.get_param('~global_roi_expand_px', 10))
        self.sor_k = int(rospy.get_param('~sor_k', 24))
        self.sor_std_mul = float(rospy.get_param('~sor_std_mul', 1.2))
        self.voxel_size_m = float(rospy.get_param('~voxel_size_m', 0.003))

        self.cyl_ransac_iters = int(rospy.get_param('~cyl_ransac_iters', 300))
        self.cyl_inlier_thresh_m = float(rospy.get_param('~cyl_inlier_thresh_m', 0.015))
        self.cyl_min_inlier_ratio = float(rospy.get_param('~cyl_min_inlier_ratio', 0.2))
        self.cyl_axis_mode = rospy.get_param('~cyl_axis_mode', 'fixed')
        self.cyl_axis_fixed_base = np.asarray(
            rospy.get_param('~cyl_axis_fixed_base', [0.0, 0.0, 1.0]),
            dtype=np.float64,
        )
        self.cyl_axis_enable_pca_fallback = bool(rospy.get_param('~cyl_axis_enable_pca_fallback', True))

        self.ray_t_min_m = float(rospy.get_param('~ray_t_min_m', 0.1))
        self.ray_t_max_m = float(rospy.get_param('~ray_t_max_m', 5.0))

        self.center_candidate_outlier_m = float(rospy.get_param('~center_candidate_outlier_m', 0.08))
        self.arc_tangent_length = float(rospy.get_param('~arc_tangent_length', 0.05))
        self.trajectory_mode = rospy.get_param('~trajectory_mode', 'dense_arc')
        self.arc_sample_step_m = float(rospy.get_param('~arc_sample_step_m', 0.004))
        self.max_arc_points = int(rospy.get_param('~max_arc_points', 500))
        self.pose_z_to_center = bool(rospy.get_param('~pose_z_to_center', True))
        # self.sort_by = rospy.get_param('~sort_by', 'arc_angle')
        self.sort_by = rospy.get_param('~sort_by', 'ocr_x')
        self.debug_image_prefix = rospy.get_param('~debug_image_prefix', 'cyl_global_surface_result')

        self.radial_reference_axis = self.safe_normalize(self.radial_reference_axis, np.array([0.0, 0.0, 1.0], dtype=np.float64))
        self.pose_reference_axis = self.safe_normalize(self.pose_reference_axis, np.array([1.0, 0.0, 0.0], dtype=np.float64))
        self.pose_reference_axis_backup = self.safe_normalize(self.pose_reference_axis_backup, np.array([0.0, 1.0, 0.0], dtype=np.float64))
        self.cyl_axis_fixed_base = self.safe_normalize(self.cyl_axis_fixed_base, np.array([0.0, 0.0, 1.0], dtype=np.float64))

        self.latest_data = {'rgb': None, 'depth': None, 'info': None}
        self.data_lock = threading.Lock()

        self.det_model_path = '/home/m/ws_moveit/PaddleOCR/new_model/det_steel_best_v2'
        self.rec_model_path = '/home/m/ws_moveit/PaddleOCR/new_model/rec_steel_best_v2'
        self.ocr = PaddleOCR(
            use_angle_cls=True,
            lang='en',
            use_gpu=False,
            det_model_dir=self.det_model_path,
            rec_model_dir=self.rec_model_path,
            rec=True,
            det_algorithm='DB',
            det_db_thresh=0.1,
            det_db_box_thresh=0.1,
            det_db_unclip_ratio=1.6,
            show_log=False,
        )

        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.script_dir = script_dir
        pkg_root = os.path.dirname(script_dir)
        self.save_dir = os.path.join(pkg_root, 'debug_images')
        os.makedirs(self.save_dir, exist_ok=True)
        self.debug_trace_txt_path = rospy.get_param(
            '~debug_trace_txt_path',
            os.path.join(self.script_dir, 'global_surface_trace.txt'),
        )

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

        rospy.loginfo('全局曲面 OCR 节点已启动, service=%s', self.service_name)
        rospy.loginfo('名义半径=%.4f m, 搜索范围=[%.4f, %.4f] m',
                      self.nominal_radius_m, self.radius_search_min_m, self.radius_search_max_m)
        rospy.loginfo('圆柱轴模式=%s, fixed_axis=%s', self.cyl_axis_mode, self.cyl_axis_fixed_base.tolist())

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

        fx, fy = float(info_msg.P[0]), float(info_msg.P[5])
        cx, cy = float(info_msg.P[2]), float(info_msg.P[6])

        # === 提取粗略的 3D 字符中心，专用于求取圆柱轴向 ===
        raw_centers_base = []
        for line in detections:
            bbox = np.asarray(line[0], dtype=np.float32)
            center_px = np.mean(bbox, axis=0)
            raw_c = self.compute_raw_center_point_from_depth(
                center_px,
                depth_arr,
                fx,
                fy,
                cx,
                cy,
                T_base_cam,
            )
            if raw_c['point_base'] is not None:
                raw_centers_base.append(raw_c['point_base'])
        # ========================================================

        global_roi = self.build_global_roi_from_detections(detections, depth_arr.shape)
        if global_roi is None:
            response.message = 'Failed to build global ROI'
            return response

        global_cloud = self.build_global_pointcloud(depth_arr, global_roi, fx, fy, cx, cy, T_base_cam)
        if global_cloud is None or len(global_cloud['points_base']) < self.min_valid_points:
            response.message = 'Global point cloud too sparse'
            return response

        points_base = self.filter_global_pointcloud(global_cloud['points_base'])
        if len(points_base) < self.min_valid_points:
            response.message = 'Global point cloud too sparse after filtering'
            return response

        cyl_model = self.fit_cylinder_model(points_base, raw_centers_base)
        if cyl_model is None:
            response.message = 'Cylinder fit failed'
            return response

        debug_image = rgb_arr.copy()
        gxmin, gxmax, gymin, gymax = global_roi
        cv2.rectangle(debug_image, (gxmin, gymin), (gxmax - 1, gymax - 1), (255, 128, 0), 2)

        valid_entries = []
        debug_rows = []
        for line in detections:
            bbox = np.asarray(line[0], dtype=np.float32)
            text = line[1][0]
            score = float(line[1][1])
            center_px = np.mean(bbox, axis=0)

            raw_center = self.compute_raw_center_point_from_depth(
                center_px=center_px,
                depth_img=depth_arr,
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                T_base_cam=T_base_cam,
            )

            result = self.compute_character_geometry_from_model(
                bbox=bbox,
                cam_info=info_msg,
                T_base_cam=T_base_cam,
                cyl_model=cyl_model,
            )

            self.draw_debug_overlay(debug_image, bbox, result['center_px'], text, score, result['debug'])

            if not result['success']:
                debug_rows.append({
                    'text': text,
                    'score': score,
                    'center_px': center_px.tolist(),
                    'raw_depth_mm': raw_center['depth_mm'],
                    'raw_point_base': raw_center['point_base'],
                    'processed_success': False,
                    'processed_point_base': None,
                    'processed_normal_base': None,
                    'fail_reason': result['debug'].get('reason', 'unknown'),
                })
                continue

            valid_entries.append({
                'text': text,
                'score': score,
                'bbox': bbox,
                'center_px': result['center_px'],
                'point_base': result['point_base'],
                'normal_base': result['normal_base'],
                'debug': result['debug'],
            })
            debug_rows.append({
                'text': text,
                'score': score,
                'center_px': center_px.tolist(),
                'raw_depth_mm': raw_center['depth_mm'],
                'raw_point_base': raw_center['point_base'],
                'processed_success': True,
                'processed_point_base': result['point_base'].tolist(),
                'processed_normal_base': result['normal_base'].tolist(),
                'fail_reason': '',
            })

        self.write_debug_trace_txt(
            rgb_msg=rgb_msg,
            info_msg=info_msg,
            T_base_cam=T_base_cam,
            global_roi=global_roi,
            cyl_model=cyl_model,
            debug_rows=debug_rows,
            valid_count=len(valid_entries),
            total_count=len(detections),
        )

        if len(valid_entries) < 2:
            response.message = 'Not enough valid chars after ray-cylinder intersection'
            self.save_debug_image(debug_image)
            return response

        if self.sort_by == 'arc_angle':
            valid_entries = self.sort_entries_by_arc_hint(valid_entries)

        circle_model = self.estimate_circle_model(valid_entries)
        if circle_model is None:
            response.message = 'Failed to estimate arc circle model'
            self.save_debug_image(debug_image)
            return response

        valid_entries = self.build_arc_poses(valid_entries, circle_model)
        if self.trajectory_mode == 'dense_arc':
            dense_poses = self.densify_arc_poses(valid_entries, circle_model)
            if not dense_poses:
                response.message = 'Dense arc generation failed'
                self.save_debug_image(debug_image)
                return response
            response.poses.extend(dense_poses)
            response.texts = ['ARC'] * len(dense_poses)
        else:
            for entry in valid_entries:
                response.poses.append(entry['pose'])
                response.texts.append(entry['text'])

        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        marker_array.markers.extend(self.build_circle_center_markers(circle_model))
        marker_array.markers.extend(self.build_cylinder_axis_markers(cyl_model))

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

        self.publish_pose_array(response.poses)
        self.marker_pub.publish(marker_array)
        self.save_debug_image(debug_image)

        response.success = len(response.poses) > 0
        response.message = 'Success: {} targets'.format(len(response.poses)) if response.success else 'No valid poses'
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
            trans_stamped = self.tf_buffer.lookup_transform(self.base_frame, camera_frame, stamp, rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            try:
                trans_stamped = self.tf_buffer.lookup_transform(self.base_frame, camera_frame, rospy.Time(0), rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as exc:
                rospy.logerr('TF Error: %s', exc)
                return None
        return self.ros_transform_to_matrix(trans_stamped.transform)

    def build_global_roi_from_detections(self, detections, image_shape):
        if len(detections) == 0:
            return None

        all_pts = []
        for det in detections:
            bbox = np.asarray(det[0], dtype=np.float32)
            all_pts.append(bbox)
        pts = np.concatenate(all_pts, axis=0)

        xmin = int(np.floor(np.min(pts[:, 0]))) - self.global_roi_expand_px
        xmax = int(np.ceil(np.max(pts[:, 0]))) + self.global_roi_expand_px
        ymin = int(np.floor(np.min(pts[:, 1]))) - self.global_roi_expand_px
        ymax = int(np.ceil(np.max(pts[:, 1]))) + self.global_roi_expand_px

        xmin = max(0, xmin)
        xmax = min(image_shape[1], xmax)
        ymin = max(0, ymin)
        ymax = min(image_shape[0], ymax)
        if xmax <= xmin or ymax <= ymin:
            return None
        return xmin, xmax, ymin, ymax

    def build_global_pointcloud(self, depth_img, global_roi, fx, fy, cx, cy, T_base_cam):
        xmin, xmax, ymin, ymax = global_roi
        roi_depth = depth_img[ymin:ymax, xmin:xmax]
        valid_mask = (roi_depth >= self.depth_min_mm) & (roi_depth <= self.depth_max_mm)
        if np.count_nonzero(valid_mask) < self.min_valid_points:
            return None

        u_grid, v_grid = np.meshgrid(np.arange(xmin, xmax), np.arange(ymin, ymax))
        u = u_grid[valid_mask].astype(np.float64)
        v = v_grid[valid_mask].astype(np.float64)
        z = roi_depth[valid_mask].astype(np.float64) / 1000.0
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points_cam = np.stack((x, y, z), axis=-1)

        R_base_cam = T_base_cam[:3, :3]
        t_base_cam = T_base_cam[:3, 3]
        points_base = (R_base_cam @ points_cam.T).T + t_base_cam[None, :]

        return {
            'points_cam': points_cam,
            'points_base': points_base,
            'u': u,
            'v': v,
        }

    def compute_raw_center_point_from_depth(self, center_px, depth_img, fx, fy, cx, cy, T_base_cam):
        u = int(round(center_px[0]))
        v = int(round(center_px[1]))
        if u < 0 or u >= depth_img.shape[1] or v < 0 or v >= depth_img.shape[0]:
            return {'depth_mm': None, 'point_base': None}

        depth_mm = float(depth_img[v, u])
        if depth_mm < self.depth_min_mm or depth_mm > self.depth_max_mm:
            return {'depth_mm': None, 'point_base': None}

        z = depth_mm / 1000.0
        x = (float(u) - cx) * z / fx
        y = (float(v) - cy) * z / fy
        point_cam = np.array([x, y, z], dtype=np.float64)

        R_base_cam = T_base_cam[:3, :3]
        t_base_cam = T_base_cam[:3, 3]
        point_base = R_base_cam @ point_cam + t_base_cam
        return {'depth_mm': depth_mm, 'point_base': point_base.tolist()}

    def filter_global_pointcloud(self, points):
        if len(points) == 0:
            return points

        filtered = points
        if self.voxel_size_m > 1e-6:
            filtered = self.voxel_downsample(filtered, self.voxel_size_m)

        if len(filtered) < max(self.sor_k + 1, self.min_valid_points):
            return filtered

        tree = cKDTree(filtered)
        dists, _ = tree.query(filtered, k=self.sor_k + 1)
        mean_d = np.mean(dists[:, 1:], axis=1)
        mu = float(np.mean(mean_d))
        sigma = float(np.std(mean_d))
        thresh = mu + self.sor_std_mul * sigma
        keep = mean_d <= thresh
        return filtered[keep]

    def voxel_downsample(self, points, voxel):
        if len(points) == 0:
            return points
        keys = np.floor(points / voxel).astype(np.int64)
        _, unique_idx = np.unique(keys, axis=0, return_index=True)
        return points[np.sort(unique_idx)]

    def fit_cylinder_model(self, points_base, raw_centers_base):
        # 1) 仅靠字符排布拓扑来获取轴向，彻底抛弃点云 PCA
        axis_dir = self.resolve_cylinder_axis_dir(raw_centers_base)
        if axis_dir is None:
            return None

        centroid = np.mean(points_base, axis=0)
        centered = points_base - centroid[None, :]

        # 2) 建立截面平面基
        basis_u = self.project_reference_to_tangent_plane(axis_dir)
        basis_v = np.cross(axis_dir, basis_u)
        basis_v = self.safe_normalize(basis_v, np.array([0.0, 0.0, 1.0], dtype=np.float64))

        # 3) 投影到截面 2D
        qx = centered @ basis_u
        qy = centered @ basis_v
        q = np.stack((qx, qy), axis=-1)

        # 4) 2D RANSAC 圆拟合（半径约束）
        model = self.fit_circle_ransac_2d(q)
        if model is None:
            return None

        c2d, radius, inliers = model
        inlier_ratio = float(np.count_nonzero(inliers)) / float(len(q))

        # 调试阶段可通过参数把阈值降到 0.2
        if inlier_ratio < self.cyl_min_inlier_ratio:
            rospy.logwarn('Cylinder inlier ratio too low: %.3f', inlier_ratio)
            return None

        axis_point = centroid + c2d[0] * basis_u + c2d[1] * basis_v

        rospy.loginfo('Cylinder fit: radius=%.4f m inliers=%d/%d ratio=%.3f',
                      radius, int(np.count_nonzero(inliers)), len(q), inlier_ratio)

        return {
            'axis_point': axis_point,
            'axis_dir': axis_dir,
            'radius': float(radius),
            'basis_u': basis_u,
            'basis_v': basis_v,
            'inlier_ratio': inlier_ratio,
        }

    def resolve_cylinder_axis_dir(self, raw_centers_base):
        pts = np.asarray(raw_centers_base, dtype=np.float64)
        if len(pts) < 3:
            rospy.logwarn('字符点太少，无法推导圆柱轴向')
            return None

        # 对字符中心点执行 PCA：字符排列成一条圆弧，圆弧平面的法向量就是圆柱轴向
        centroid = np.mean(pts, axis=0)
        centered = pts - centroid[None, :]
        cov = centered.T @ centered / max(len(pts) - 1, 1)
        eigvals, eigvecs = np.linalg.eigh(cov)

        # 最小特征向量即为圆弧平面法向（圆柱轴）
        axis_dir = eigvecs[:, np.argmin(eigvals)]

        # 施加物理约束：强行让它平行于 base_link 的 X-Y 平面 (消除俯仰误差)
        axis_dir[2] = 0.0
        axis_dir = self.safe_normalize(axis_dir, np.array([1.0, 0.0, 0.0], dtype=np.float64))

        # 统一轴的正负方向（让 X 始终为正，防反转）
        if axis_dir[0] < 0:
            axis_dir = -axis_dir

        rospy.loginfo('结合字符拓扑推导出的绝对轴向: [%.4f, %.4f, %.4f]',
                      axis_dir[0], axis_dir[1], axis_dir[2])
        return axis_dir

    def estimate_axis_dir_by_pca(self, centered):
        if len(centered) < 3:
            return None
        cov = centered.T @ centered / max(len(centered) - 1, 1)
        eigvals, eigvecs = np.linalg.eigh(cov)
        axis_dir = eigvecs[:, np.argmax(eigvals)]
        axis_dir = self.safe_normalize(axis_dir, self.cyl_axis_fixed_base)
        if np.dot(axis_dir, self.cyl_axis_fixed_base) < 0.0:
            axis_dir = -axis_dir
        return axis_dir

    def fit_circle_ransac_2d(self, q):
        n = len(q)
        if n < 8:
            return None

        best_inliers = None
        best_center = None
        best_radius = None
        rng = np.random.default_rng(int(time.time() * 1000) % (2**32 - 1))

        for _ in range(self.cyl_ransac_iters):
            idx = rng.choice(n, size=3, replace=False)
            c, r = self.circle_from_3pts(q[idx[0]], q[idx[1]], q[idx[2]])
            if c is None:
                continue
            if r < self.radius_search_min_m or r > self.radius_search_max_m:
                continue

            residual = np.abs(np.linalg.norm(q - c[None, :], axis=1) - r)
            inliers = residual <= self.cyl_inlier_thresh_m
            if best_inliers is None or np.count_nonzero(inliers) > np.count_nonzero(best_inliers):
                best_inliers = inliers
                best_center = c
                best_radius = r

        if best_inliers is None or np.count_nonzero(best_inliers) < 6:
            return None

        # 用内点做线性圆拟合（Kasa）细化
        q_in = q[best_inliers]
        c_refine, r_refine = self.circle_fit_kasa(q_in)
        if c_refine is None:
            c_refine, r_refine = best_center, best_radius

        if r_refine < self.radius_search_min_m or r_refine > self.radius_search_max_m:
            c_refine, r_refine = best_center, best_radius

        return c_refine, float(r_refine), best_inliers

    def circle_from_3pts(self, p1, p2, p3):
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3

        A = np.array([
            [x2 - x1, y2 - y1],
            [x3 - x1, y3 - y1],
        ], dtype=np.float64)
        b = 0.5 * np.array([
            x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1,
            x3 * x3 + y3 * y3 - x1 * x1 - y1 * y1,
        ], dtype=np.float64)

        det = np.linalg.det(A)
        if abs(det) < 1e-9:
            return None, None

        c = np.linalg.solve(A, b)
        r = np.linalg.norm(p1 - c)
        return c, float(r)

    def circle_fit_kasa(self, q):
        if len(q) < 3:
            return None, None
        x = q[:, 0]
        y = q[:, 1]
        A = np.column_stack((2.0 * x, 2.0 * y, np.ones_like(x)))
        b = x * x + y * y
        try:
            sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        except np.linalg.LinAlgError:
            return None, None
        cx, cy, c0 = sol
        r2 = cx * cx + cy * cy + c0
        if r2 <= 0.0:
            return None, None
        return np.array([cx, cy], dtype=np.float64), float(np.sqrt(r2))

    def compute_character_geometry_from_model(self, bbox, cam_info, T_base_cam, cyl_model):
        fx, fy = float(cam_info.P[0]), float(cam_info.P[5])
        cx, cy = float(cam_info.P[2]), float(cam_info.P[6])

        center_px = np.mean(bbox, axis=0)
        ray_cam = self.pixel_to_ray(center_px[0], center_px[1], fx, fy, cx, cy)

        R_base_cam = T_base_cam[:3, :3]
        t_base_cam = T_base_cam[:3, 3]
        ray_origin = t_base_cam
        ray_dir = R_base_cam @ ray_cam
        ray_dir = self.safe_normalize(ray_dir, np.array([0.0, 0.0, 1.0], dtype=np.float64))

        point_base = self.intersect_ray_cylinder(ray_origin, ray_dir, cyl_model)
        if point_base is None:
            return {
                'success': False,
                'center_px': center_px,
                'debug': {'reason': 'ray_cylinder_no_intersection'}
            }

        axis_point = cyl_model['axis_point']
        axis_dir = cyl_model['axis_dir']
        closest_axis_pt = axis_point + np.dot(point_base - axis_point, axis_dir) * axis_dir
        normal = point_base - closest_axis_pt
        normal = self.safe_normalize(normal, self.radial_reference_axis)

        if np.dot(normal, self.radial_reference_axis) < 0.0:
            normal = -normal

        return {
            'success': True,
            'center_px': center_px,
            'point_base': point_base,
            'normal_base': normal,
            'debug': {
                'center_source': 'ray_cylinder',
                'radius_fit': cyl_model['radius'],
                'normal_alignment': float(np.dot(normal, self.radial_reference_axis)),
            }
        }

    def intersect_ray_cylinder(self, ray_origin, ray_dir, cyl_model):
        c = cyl_model['axis_point']
        a = cyl_model['axis_dir']
        r = float(cyl_model['radius'])

        m = ray_origin - c
        n = ray_dir
        m_perp = m - np.dot(m, a) * a
        n_perp = n - np.dot(n, a) * a

        A = float(np.dot(n_perp, n_perp))
        B = float(2.0 * np.dot(m_perp, n_perp))
        C = float(np.dot(m_perp, m_perp) - r * r)

        if A < 1e-12:
            return None

        disc = B * B - 4.0 * A * C
        if disc < 0.0:
            return None

        sqrt_disc = np.sqrt(max(disc, 0.0))
        t1 = (-B - sqrt_disc) / (2.0 * A)
        t2 = (-B + sqrt_disc) / (2.0 * A)
        candidates = [t for t in (t1, t2) if t > self.ray_t_min_m and t < self.ray_t_max_m]
        if not candidates:
            return None
        t = min(candidates)
        return ray_origin + t * ray_dir

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

        plane_origin = np.mean(points, axis=0)
        centered_points = points - plane_origin[None, :]
        cov = centered_points.T @ centered_points / max(len(points) - 1, 1)
        eigvals, eigvecs = np.linalg.eigh(cov)
        plane_normal = eigvecs[:, np.argmin(eigvals)]
        plane_normal = self.safe_normalize(plane_normal, np.array([0.0, 1.0, 0.0], dtype=np.float64))

        center_on_plane = center_fused - np.dot(center_fused - plane_origin, plane_normal) * plane_normal

        return {
            'center': center_on_plane,
            'plane_normal': plane_normal,
            'plane_origin': plane_origin,
            'radius': self.nominal_radius_m,
        }

    def build_arc_poses(self, entries, circle_model):
        center = circle_model['center']
        plane_normal = circle_model['plane_normal']
        plane_origin = circle_model['plane_origin']
        radius = circle_model['radius']

        projected_points = []
        previous_tangent = None
        previous_quat = None

        for entry in entries:
            point = entry['point_base']
            point_on_plane = point - np.dot(point - plane_origin, plane_normal) * plane_normal
            radial = point_on_plane - center
            radial = self.safe_normalize(radial, entry['normal_base'])
            point_on_circle = center + radius * radial
            entry['projected_point_base'] = point_on_circle
            entry['radial_out_base'] = radial
            projected_points.append(point_on_circle)

        projected_points = np.asarray(projected_points, dtype=np.float64)
        for index, entry in enumerate(entries):
            target_z = self.get_target_z_from_radial(entry['radial_out_base'])
            tangent = self.estimate_arc_tangent(index, projected_points)
            tangent = tangent - np.dot(tangent, target_z) * target_z
            tangent = self.safe_normalize(tangent, self.project_reference_to_tangent_plane(target_z))

            if previous_tangent is not None and np.dot(tangent, previous_tangent) < 0.0:
                tangent = -tangent
            previous_tangent = tangent

            target_x = tangent
            target_y = np.cross(target_z, target_x)
            target_y = self.safe_normalize(target_y, np.array([0.0, 1.0, 0.0], dtype=np.float64))
            target_x = np.cross(target_y, target_z)
            target_x = self.safe_normalize(target_x, self.pose_reference_axis)

            rot = np.column_stack((target_x, target_y, target_z))
            quat = R.from_matrix(rot).as_quat()
            if previous_quat is not None and np.dot(quat, previous_quat) < 0.0:
                quat = -quat
            previous_quat = quat

            pose = Pose()
            pose.position.x = float(entry['projected_point_base'][0])
            pose.position.y = float(entry['projected_point_base'][1])
            pose.position.z = float(entry['projected_point_base'][2])
            pose.orientation.x = float(quat[0])
            pose.orientation.y = float(quat[1])
            pose.orientation.z = float(quat[2])
            pose.orientation.w = float(quat[3])
            entry['pose'] = pose

        return entries

    def densify_arc_poses(self, entries, circle_model):
        if len(entries) <= 1:
            return [copy.deepcopy(entries[0]['pose'])] if len(entries) == 1 else []

        center = circle_model['center']
        plane_normal = circle_model['plane_normal']
        radius = float(circle_model['radius'])
        if radius <= 1e-6:
            return []

        radial0 = self.safe_normalize(entries[0]['radial_out_base'], np.array([1.0, 0.0, 0.0], dtype=np.float64))
        basis_u = radial0
        basis_v = np.cross(plane_normal, basis_u)
        basis_v = self.safe_normalize(basis_v, self.pose_reference_axis_backup)

        raw_angles = []
        for entry in entries:
            radial = entry['radial_out_base']
            raw_angles.append(float(np.arctan2(np.dot(radial, basis_v), np.dot(radial, basis_u))))

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
            count = max(1, int(np.ceil(abs(delta) / theta_step)))
            for i in range(1, count + 1):
                dense_angles.append(start_theta + delta * (float(i) / float(count)))

        if len(dense_angles) > self.max_arc_points:
            sample_idx = np.linspace(0, len(dense_angles) - 1, self.max_arc_points).astype(np.int32)
            dense_angles = [dense_angles[i] for i in sample_idx]

        direction_sign = 1.0 if (unwrapped[-1] - unwrapped[0]) >= 0.0 else -1.0
        previous_tangent = None
        previous_quat = None
        dense_poses = []

        for theta in dense_angles:
            radial = np.cos(theta) * basis_u + np.sin(theta) * basis_v
            radial = self.safe_normalize(radial, basis_u)
            point = center + radius * radial

            target_z = self.get_target_z_from_radial(radial)
            tangent = direction_sign * (-np.sin(theta) * basis_u + np.cos(theta) * basis_v)
            tangent = tangent - np.dot(tangent, target_z) * target_z
            tangent = self.safe_normalize(tangent, self.project_reference_to_tangent_plane(target_z))

            if previous_tangent is not None and np.dot(tangent, previous_tangent) < 0.0:
                tangent = -tangent
            previous_tangent = tangent

            target_x = tangent
            target_y = np.cross(target_z, target_x)
            target_y = self.safe_normalize(target_y, self.pose_reference_axis_backup)
            target_x = np.cross(target_y, target_z)
            target_x = self.safe_normalize(target_x, self.pose_reference_axis)

            rot = np.column_stack((target_x, target_y, target_z))
            quat = R.from_matrix(rot).as_quat()
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

        return dense_poses

    def get_target_z_from_radial(self, radial_out):
        return -radial_out if self.pose_z_to_center else radial_out

    def estimate_arc_tangent(self, index, projected_points):
        if len(projected_points) <= 1:
            return self.pose_reference_axis.copy()
        if index == 0:
            tangent = projected_points[1] - projected_points[0]
        elif index == len(projected_points) - 1:
            tangent = projected_points[-1] - projected_points[-2]
        else:
            tangent = projected_points[index + 1] - projected_points[index - 1]
        return self.safe_normalize(tangent, self.pose_reference_axis)

    def sort_entries_by_arc_hint(self, entries):
        points = np.asarray([entry['point_base'] for entry in entries], dtype=np.float64)
        plane_origin = np.mean(points, axis=0)
        centered = points - plane_origin[None, :]
        cov = centered.T @ centered / max(len(points) - 1, 1)
        _, eigvecs = np.linalg.eigh(cov)
        basis_u = eigvecs[:, -1]
        basis_u = basis_u - np.dot(basis_u, self.radial_reference_axis) * self.radial_reference_axis
        basis_u = self.safe_normalize(basis_u, self.project_reference_to_tangent_plane(self.radial_reference_axis))
        basis_v = np.cross(self.radial_reference_axis, basis_u)
        basis_v = self.safe_normalize(basis_v, self.pose_reference_axis_backup)

        for entry in entries:
            p = entry['point_base'] - plane_origin
            entry['arc_angle_hint'] = float(np.arctan2(np.dot(p, basis_v), np.dot(p, basis_u)))
        return sorted(entries, key=lambda it: it['arc_angle_hint'])

    def pixel_to_ray(self, u, v, fx, fy, cx, cy):
        ray = np.array([
            (float(u) - cx) / fx,
            (float(v) - cy) / fy,
            1.0,
        ], dtype=np.float64)
        return self.safe_normalize(ray, np.array([0.0, 0.0, 1.0], dtype=np.float64))

    def project_reference_to_tangent_plane(self, target_z):
        for axis in (self.pose_reference_axis, self.pose_reference_axis_backup, np.array([0.0, 0.0, 1.0], dtype=np.float64)):
            tangent = axis - np.dot(axis, target_z) * target_z
            if np.linalg.norm(tangent) > 1e-6:
                return tangent / np.linalg.norm(tangent)
        return np.array([1.0, 0.0, 0.0], dtype=np.float64)

    def safe_normalize(self, v, fallback):
        n = np.linalg.norm(v)
        if n < 1e-9:
            return fallback.copy()
        return v / n

    def draw_debug_overlay(self, image, bbox, center_px, text, score, debug_payload):
        box_int = np.round(bbox).astype(np.int32)
        cv2.polylines(image, [box_int], isClosed=True, color=(0, 255, 0), thickness=2)
        cx, cy = int(round(center_px[0])), int(round(center_px[1]))
        cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)
        label = '{} {:.2f}'.format(text, score)
        cv2.putText(image, label, (box_int[0][0], max(20, box_int[0][1] - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)
        if 'radius_fit' in debug_payload:
            info = 'R={:.3f} na={:.3f}'.format(
                debug_payload.get('radius_fit', 0.0),
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

    def build_cylinder_axis_markers(self, cyl_model):
        markers = []
        c = cyl_model['axis_point']
        a = cyl_model['axis_dir']

        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'cylinder_axis'
        marker.id = 9100
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        p0 = Point(x=float(c[0] - a[0] * 0.2), y=float(c[1] - a[1] * 0.2), z=float(c[2] - a[2] * 0.2))
        p1 = Point(x=float(c[0] + a[0] * 0.2), y=float(c[1] + a[1] * 0.2), z=float(c[2] + a[2] * 0.2))
        marker.points = [p0, p1]
        markers.append(marker)
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

    def write_debug_trace_txt(self, rgb_msg, info_msg, T_base_cam, global_roi, cyl_model, debug_rows, valid_count, total_count):
        try:
            cam_pos = T_base_cam[:3, 3].tolist()
            cam_rot = R.from_matrix(T_base_cam[:3, :3]).as_quat().tolist()
            lines = []
            lines.append('============================================================')
            lines.append('timestamp_ms: {}'.format(int(time.time() * 1000)))
            lines.append('rgb_frame_id: {}'.format(rgb_msg.header.frame_id))
            lines.append('camera_info_frame_id: {}'.format(info_msg.header.frame_id))
            lines.append('base_frame: {}'.format(self.base_frame))
            lines.append('camera_position_in_base[m]: [{:.6f}, {:.6f}, {:.6f}]'.format(*cam_pos))
            lines.append('camera_quat_in_base[xyzw]: [{:.6f}, {:.6f}, {:.6f}, {:.6f}]'.format(*cam_rot))
            lines.append('global_roi[xmin,xmax,ymin,ymax]: {}'.format(list(global_roi)))
            lines.append('detections_total: {}  valid_after_model: {}'.format(total_count, valid_count))
            lines.append('cylinder_axis_point[m]: [{:.6f}, {:.6f}, {:.6f}]'.format(*cyl_model['axis_point'].tolist()))
            lines.append('cylinder_axis_dir: [{:.6f}, {:.6f}, {:.6f}]'.format(*cyl_model['axis_dir'].tolist()))
            lines.append('cylinder_radius_fit[m]: {:.6f}  inlier_ratio: {:.4f}'.format(
                float(cyl_model['radius']),
                float(cyl_model.get('inlier_ratio', -1.0)),
            ))
            lines.append('--- per char ---')

            for idx, row in enumerate(debug_rows):
                lines.append('[{}] text={} score={:.3f}'.format(idx, row['text'], float(row['score'])))
                lines.append('  center_px: [{:.2f}, {:.2f}]'.format(float(row['center_px'][0]), float(row['center_px'][1])))
                if row['raw_depth_mm'] is None or row['raw_point_base'] is None:
                    lines.append('  raw_depth_center: invalid')
                else:
                    rp = row['raw_point_base']
                    lines.append('  raw_depth_mm: {:.2f}'.format(float(row['raw_depth_mm'])))
                    lines.append('  raw_point_base[m]: [{:.6f}, {:.6f}, {:.6f}]'.format(float(rp[0]), float(rp[1]), float(rp[2])))

                if row['processed_success']:
                    pp = row['processed_point_base']
                    pn = row['processed_normal_base']
                    lines.append('  processed_point_base[m]: [{:.6f}, {:.6f}, {:.6f}]'.format(float(pp[0]), float(pp[1]), float(pp[2])))
                    lines.append('  processed_normal_base: [{:.6f}, {:.6f}, {:.6f}]'.format(float(pn[0]), float(pn[1]), float(pn[2])))
                    if row['raw_point_base'] is not None:
                        rpv = np.array(row['raw_point_base'], dtype=np.float64)
                        ppv = np.array(row['processed_point_base'], dtype=np.float64)
                        diff = ppv - rpv
                        lines.append('  delta_processed_minus_raw[m]: [{:.6f}, {:.6f}, {:.6f}] |norm|={:.6f}'.format(
                            float(diff[0]), float(diff[1]), float(diff[2]), float(np.linalg.norm(diff))
                        ))
                else:
                    lines.append('  processed: failed reason={}'.format(row['fail_reason']))

            with open(self.debug_trace_txt_path, 'a', encoding='utf-8') as fp:
                fp.write('\n'.join(lines) + '\n')

            rospy.loginfo('Debug trace txt saved: %s', self.debug_trace_txt_path)
        except Exception as exc:
            rospy.logwarn('Failed to write debug trace txt: %s', exc)

    def ros_transform_to_matrix(self, trans):
        q = trans.rotation
        t = trans.translation
        mat = np.eye(4)
        mat[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        mat[:3, 3] = [t.x, t.y, t.z]
        return mat


if __name__ == '__main__':
    try:
        server = SteelStampCylindricalGlobalSurfaceServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

