#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ================== 【关键修改 1/2】在最顶部强制设置环境变量并预导入 sklearn ==================
import os
import sys

# 强制预加载系统 libgomp (请确认路径与你系统上的一致，可用 find /usr/lib -name "libgomp.so.1" 查找)
# 注意：这行代码必须在 import 任何其他库（包括 scipy, cv2, paddleocr）之前执行
# os.environ["LD_PRELOAD"] = "/usr/lib/aarch64-linux-gnu/libgomp.so.1"
os.environ["LD_PRELOAD"] = "/usr/lib/x86_64-linux-gnu/libgomp.so.1"

# 强制最先导入 sklearn，让它在其他库之前占用 TLS 内存
try:
    import sklearn
    print(f"[DEBUG] 成功预导入 sklearn: {sklearn.__version__}")
except ImportError as e:
    print(f"[WARN] sklearn 预导入失败: {e}")
# ==============================================================================================

# 接下来才是原本的导入，但顺序稍微调整一下
import copy
import threading
import time

# 注意：先把 ROS 相关的 sys.path 处理好，但不要在这里导入具体 ROS 库
ros_path = '/opt/ros/noetic/lib/python3/dist-packages'
sys_path = '/usr/lib/python3/dist-packages'

if os.path.exists(ros_path) and ros_path not in sys.path:
    sys.path.append(ros_path)

if os.path.exists(sys_path) and sys_path not in sys.path:
    sys.path.append(sys_path)

# ================== 【关键修改 2/2】把 paddleocr 的导入尽量提前，放在 cv2 等之后 ==================
import cv2
import numpy as np
from paddleocr import PaddleOCR  # <--- 把 PaddleOCR 移到这里，紧跟在基础库之后
# ==============================================================================================

# 剩下的原本的导入
import message_filters
import rospy
import tf2_ros
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, Pose, PoseArray
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as SensorImage
from visualization_msgs.msg import Marker, MarkerArray

from mv3d_rgbd_ros.srv import DetectSteelStamp, DetectSteelStampResponse


class SteelStampCylindricalGlobalSurfaceServer:
    """
    全局曲面版钢印 OCR 节点：
    1) OCR 识别字符框
    2) 合并字符框得到全局 ROI，提取宏观点云
    3) 对全局点云做 SOR 去飞点
    4) 【修改】提取局部点云法向，结合绝对先验半径和先验轴向，直接推导圆柱曲面模型
    5) 对每个字符中心像素做“射线-圆柱求交”，得到稳定 3D 点与法向
    6) 基于字符 3D 点生成平滑圆弧轨迹与姿态
    """

    def __init__(self):
        rospy.init_node('steel_stamp_cylindrical_global_surface_server_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()

        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.service_name = rospy.get_param('~service_name', 'get_steel_stamp_location')
        self.rgb_topic = rospy.get_param('~rgb_topic', '/camera/rgb/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/rgb/camera_info')

        self.sync_queue_size = int(rospy.get_param('~sync_queue_size', 10))
        self.sync_slop = float(rospy.get_param('~sync_slop', 0.05))
        #根据高度来填，.min_valid_points看情况适当调低
        self.depth_min_mm = int(rospy.get_param('~depth_min_mm', 500))
        self.depth_max_mm = int(rospy.get_param('~depth_max_mm', 1000))
        self.min_valid_points = int(rospy.get_param('~min_valid_points', 200))

        # 核心先验参数：名义直径和半径
        self.nominal_diameter_m = float(rospy.get_param('~nominal_diameter_m', 4.3))
        self.nominal_radius_m = 0.5 * self.nominal_diameter_m

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

        # RANSAC 参数已废弃，保留固定轴向配置
        #可能需要更改
        # [1,0,0]、[0,1,0]
        self.cyl_axis_mode = rospy.get_param('~cyl_axis_mode', 'fixed')
        self.cyl_axis_fixed_base = np.asarray(
            rospy.get_param('~cyl_axis_fixed_base', [1.0, 0.0, 0.0]),
            dtype=np.float64,
        )

        self.ray_t_min_m = float(rospy.get_param('~ray_t_min_m', 0.1))
        self.ray_t_max_m = float(rospy.get_param('~ray_t_max_m', 5.0))

        self.center_candidate_outlier_m = float(rospy.get_param('~center_candidate_outlier_m', 0.08))
        self.arc_tangent_length = float(rospy.get_param('~arc_tangent_length', 0.05))
        self.trajectory_mode = rospy.get_param('~trajectory_mode', 'dense_arc')
        self.arc_sample_step_m = float(rospy.get_param('~arc_sample_step_m', 0.004))
        self.max_arc_points = int(rospy.get_param('~max_arc_points', 500))
        self.trajectory_normal_offset_m = float(rospy.get_param('~trajectory_normal_offset_m', -0.035))
        # 仅用于“曲面解释”内/外凹，不改变姿态朝向约定
        # 可选: outer/convex(外凸, 默认), inner/concave(内凹)
        self.surface_mode = str(rospy.get_param('~surface_mode', 'inner')).strip().lower()
        if self.surface_mode in ('inner', 'concave'):
            self.surface_center_sign = 1.0
        elif self.surface_mode in ('outer', 'convex'):
            self.surface_center_sign = -1.0
        else:
            rospy.logwarn('Unknown ~surface_mode=%s, fallback to outer', self.surface_mode)
            self.surface_mode = 'outer'
            self.surface_center_sign = -1.0
        self.pose_z_to_center = bool(rospy.get_param('~pose_z_to_center', False))
        self.sort_by = rospy.get_param('~sort_by', 'ocr_x')
        self.debug_image_prefix = rospy.get_param('~debug_image_prefix', 'cyl_global_surface_result')

        self.radial_reference_axis = self.safe_normalize(self.radial_reference_axis, np.array([0.0, 0.0, 1.0], dtype=np.float64))
        self.pose_reference_axis = self.safe_normalize(self.pose_reference_axis, np.array([1.0, 0.0, 0.0], dtype=np.float64))
        self.pose_reference_axis_backup = self.safe_normalize(self.pose_reference_axis_backup, np.array([0.0, 1.0, 0.0], dtype=np.float64))
        ##指带base_link的x方向
        self.cyl_axis_fixed_base = self.safe_normalize(self.cyl_axis_fixed_base, np.array([1.0, 0.0, 0.0], dtype=np.float64))

        self.latest_data = {'rgb': None, 'depth': None, 'info': None}
        self.data_lock = threading.Lock()
        self.bridge = CvBridge()

        self.det_model_path = '/home/barry/workspace/ws_moveit/PaddleOCR/inference/det_steel_wall_new'
        self.rec_model_path = '/home/barry/workspace/ws_moveit/PaddleOCR/inference/steel_rec_model'
        self.ocr = PaddleOCR(
            use_angle_cls=True,
            lang='en',
            use_gpu=True,
            det_model_dir=self.det_model_path,
            rec_model_dir=self.rec_model_path,
            rec=True,
            det_algorithm='DB',
            det_db_thresh=0.3,
            det_db_box_thresh=0.3,
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
        rospy.loginfo('固定名义半径=%.4f m', self.nominal_radius_m)
        rospy.loginfo('圆柱绝对先验轴向= %s', self.cyl_axis_fixed_base.tolist())
        rospy.loginfo('曲面解释模式 surface_mode=%s (center_sign=%.1f)', self.surface_mode, self.surface_center_sign)

    def sync_callback(self, rgb_msg, depth_msg, info_msg):
        with self.data_lock:
            self.latest_data['rgb'] = rgb_msg
            self.latest_data['depth'] = depth_msg
            self.latest_data['info'] = info_msg

    def handle_req(self, _req):
        # =====================================================================
        # handle_req — ROS 服务回调，全局圆柱曲面版 OCR + 轨迹生成的核心流程。
        #
        # 与 solo 版的核心区别：
        #   solo 对每个字符独立抠 21×21 小窗口，局部 PCA 求法向量；
        #   本版本将所有字符合并为一个"全局 ROI"，提取整片点云，
        #   利用"钢件是圆柱面"的先验知识，拟合圆柱模型，再通过
        #   "射线-圆柱求交"精确定位每个字符的 3D 位置和法向量。
        #
        # 整体管线：
        #   同步图像获取 → 新鲜度检查 → 图像解码 → PaddleOCR 检测+识别
        #   → 按 x 排序字符 → TF 查询相机→基座变换
        #   → 【阶段 A：圆柱建模】
        #       A1. 每个字符中心像素 → 5×5 窗口中位数深度 → 粗略 3D 中心
        #       A2. 所有字符合并 → 全局 ROI → 提取大片点云
        #       A3. 体素降采样 + SOR 统计滤波去噪
        #       A4. 2D 投影 + 非线性最小二乘拟合圆柱（固定轴向 + 固定名义半径）
        #   → 【阶段 B：字符定位】
        #       B1. 每个字符中心像素 → 相机射线
        #       B2. 射线-圆柱求交 → 精确 3D 位置 + 径向法向量
        #       B3. 绘制调试叠加图
        #   → 【阶段 C：轨迹生成】
        #       C1. 从字符法向和位置估算圆弧中心（截面圆模型）
        #       C2. 沿圆柱面展开圆弧姿态
        #       C3. 密度圆弧插值（dense_arc 模式）
        #       C4. 法向偏移（工具长度补偿）
        #   → 发布 RViz 可视化 + 保存调试图片 + 写追踪日志
        #   → 返回 {success, message, texts[], poses[]}
        #
        # 参数:
        #   _req: DetectSteelStamp 服务请求（空请求，仅用于触发）
        #
        # 返回:
        #   DetectSteelStampResponse:
        #     - success (bool):   至少 1 个有效姿态
        #     - message (string): 状态描述
        #     - texts   (string[]): 识别文本（dense_arc 模式统一填 'ARC'）
        #     - poses   (Pose[]):   最终轨迹姿态
        # =====================================================================

        # ------------------------------------------------------------------
        # 1. 初始化响应对象，悲观设 success=False（防御性编程）
        # ------------------------------------------------------------------
        response = DetectSteelStampResponse()
        response.success = False

        # ------------------------------------------------------------------
        # 2. 线程安全读取最新同步数据
        # ------------------------------------------------------------------
        with self.data_lock:
            rgb_msg   = self.latest_data['rgb']    # RGB 彩色图像消息
            depth_msg = self.latest_data['depth']  # 深度图 (uint16, 毫米)
            info_msg  = self.latest_data['info']   # 相机内参

        # ------------------------------------------------------------------
        # 3. 数据就绪检查
        # ------------------------------------------------------------------
        if rgb_msg is None or depth_msg is None or info_msg is None:
            response.message = 'No synchronized RGB/Depth/CameraInfo received yet'
            return response

        # ------------------------------------------------------------------
        # 4. 数据新鲜度检查
        #    如果图像缓存超过 1 秒，说明相机可能断流或处理拥塞，快速失败
        # ------------------------------------------------------------------
        latency = (rospy.Time.now() - rgb_msg.header.stamp).to_sec()
        if latency > 1.0:
            response.message = 'Data is too old (latency: {:.2f}s). Camera stream might be stalled.'.format(latency)
            rospy.logerr(response.message)
            return response

        # ------------------------------------------------------------------
        # 5. ROS 图像消息 → OpenCV numpy 数组
        #    rgb_arr:   (H,W,3) uint8  BGR 格式
        #    depth_arr: (H,W)   uint16 毫米单位
        # ------------------------------------------------------------------
        try:
            rgb_arr   = self.bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding='bgr8')
            depth_arr = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            if depth_arr.dtype != np.uint16:
                depth_arr = depth_arr.astype(np.uint16, copy=False)
        except (CvBridgeError, Exception) as exc:
            response.message = 'Image parse error: {}'.format(exc)
            rospy.logerr(response.message)
            return response

        # ------------------------------------------------------------------
        # 6. PaddleOCR 检测 + 识别
        #    返回格式: ocr_results = [[[四点框, [文本, 置信度]], ...]]
        #    单张图片输入，取 ocr_results[0] 得到所有检测框列表
        # ------------------------------------------------------------------
        ocr_results = self.ocr.ocr(rgb_arr, cls=False)
        if not ocr_results or not ocr_results[0]:
            response.message = 'OCR detected nothing'
            return response

        # ------------------------------------------------------------------
        # 7. 检测框按 x 坐标从左到右排序
        #    item[0] = 四点框 [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
        #    np.mean(..., axis=0) → 中心点 [cx, cy]
        #    [:, 0] → 取 x 坐标排序
        # ------------------------------------------------------------------
        detections = ocr_results[0]
        detections.sort(key=lambda item: np.mean(np.asarray(item[0], dtype=np.float32)[:, 0]))

        # ------------------------------------------------------------------
        # 8. TF 查询：相机坐标系 → 机器人基座坐标系的 4×4 齐次变换矩阵
        # ------------------------------------------------------------------
        T_base_cam = self.lookup_camera_transform(rgb_msg)
        if T_base_cam is None:
            response.message = 'TF lookup failed'
            return response

        # ------------------------------------------------------------------
        # 9. 解析相机投影矩阵 P，提取内参 (fx, fy, cx, cy)
        # ------------------------------------------------------------------
        fx, fy = float(info_msg.P[0]), float(info_msg.P[5])
        cx, cy = float(info_msg.P[2]), float(info_msg.P[6])

        # ==================================================================
        # 【阶段 A：圆柱建模】
        # ==================================================================

        # --- A1. 提取每个字符的粗略 3D 中心（供后续轴向推导用）---
        # 对每个 OCR 检测框的中心像素，在 5×5 窗口中取中位数深度，
        # 然后通过针孔模型反投影到 3D 相机坐标，再转换到基座坐标系。
        # 这些"粗略中心"不用于最终定位，仅用于辅助推导圆柱轴向。
        raw_centers_base = []
        for line in detections:
            bbox = np.asarray(line[0], dtype=np.float32)
            center_px = np.mean(bbox, axis=0)
            raw_c = self.compute_raw_center_point_from_depth(
                center_px, depth_arr, fx, fy, cx, cy, T_base_cam
            )
            if raw_c['point_base'] is not None:
                raw_centers_base.append(raw_c['point_base'])

        # --- A2. 构建全局 ROI 并提取宏观点云 ---
        # 将所有字符检测框合并为一个大的包围矩形（加 expand 边距），
        # 一次性从深度图中提取整片区域的点云数据。
        # 与 solo 逐字符抠 21×21 小窗口的思路截然不同——
        # 这里追求的是"覆盖整个钢印区域的大片点云"。
        global_roi = self.build_global_roi_from_detections(detections, depth_arr.shape)
        if global_roi is None:
            response.message = 'Failed to build global ROI'
            return response

        global_cloud = self.build_global_pointcloud(depth_arr, global_roi, fx, fy, cx, cy, T_base_cam)
        if global_cloud is None or len(global_cloud['points_base']) < self.min_valid_points:
            response.message = 'Global point cloud too sparse'
            return response

        # --- A3. 点云滤波：体素降采样 + SOR 统计去噪 ---
        # 体素降采样：将点云按 3mm 网格合并，减少点数加速后续处理
        # SOR：对每个点计算到 k=24 个邻居的平均距离，
        #       剔除距离异常的飞点（反光导致的深度跳变）
        points_base = self.filter_global_pointcloud(global_cloud['points_base'])
        if len(points_base) < self.min_valid_points:
            response.message = 'Global point cloud too sparse after filtering'
            return response

        # --- A4. 圆柱拟合（核心数学）---
        # 使用固定物理先验：轴向 = cyl_axis_fixed_base（默认 [1,0,0]，沿 X 轴）
        #                    半径 = nominal_radius_m（已知加工参数）
        # 方法：将 3D 点云投影到垂直于轴向的 2D 平面，
        #       用非线性最小二乘（Levenberg-Marquardt）精确求解最优圆心。
        #       返回 {axis_point, axis_dir, radius, basis_u, basis_v}
        cyl_model = self.fit_cylinder_model(points_base, raw_centers_base)
        if cyl_model is None:
            response.message = 'Cylinder fit failed'
            return response

        # ==================================================================
        # 【阶段 B：字符精确定位】
        # ==================================================================

        # 创建调试用图像副本，在全局 ROI 画蓝色矩形框
        debug_image = rgb_arr.copy()
        gxmin, gxmax, gymin, gymax = global_roi
        cv2.rectangle(debug_image, (gxmin, gymin), (gxmax - 1, gymax - 1), (255, 128, 0), 2)

        valid_entries = []   # 成功定位的字符列表
        debug_rows = []      # 调试追踪数据（每个字符一行记录）
        for line in detections:
            bbox  = np.asarray(line[0], dtype=np.float32)  # 四点检测框
            text  = line[1][0]                              # 识别字符
            score = float(line[1][1])                       # 置信度
            center_px = np.mean(bbox, axis=0)               # 框中心像素

            # --- B1. 获取原始深度（5×5 窗口中位数，仅用于调试对比）---
            raw_center = self.compute_raw_center_point_from_depth(
                center_px=center_px, depth_img=depth_arr,
                fx=fx, fy=fy, cx=cx, cy=cy, T_base_cam=T_base_cam,
            )

            # --- B2. 射线-圆柱求交：字符的精确 3D 定位 ---
            # 从相机光心发出一条穿过字符中心像素的射线，
            # 与该射线与圆柱曲面求交，交点即为字符的精确 3D 位置。
            # 法向量 = 交点指向圆柱轴线的径向方向。
            # 这种方法比 solo 的"局部窗口重心"更精确——
            # 因为交点严格落在圆柱面上，而不是近似平面重心。
            result = self.compute_character_geometry_from_model(
                bbox=bbox, cam_info=info_msg, T_base_cam=T_base_cam, cyl_model=cyl_model,
            )

            # --- B3. 在调试图上绘制检测框、字符、置信度 ---
            self.draw_debug_overlay(debug_image, bbox, result['center_px'], text, score, result['debug'])

            if not result['success']:
                # 射线-圆柱求交失败（如射线的垂线与圆柱无交点）
                debug_rows.append({
                    'text': text, 'score': score, 'center_px': center_px.tolist(),
                    'raw_depth_mm': raw_center['depth_mm'],
                    'raw_point_base': raw_center['point_base'],
                    'processed_success': False,
                    'processed_point_base': None,
                    'processed_normal_base': None,
                    'fail_reason': result['debug'].get('reason', 'unknown'),
                })
                continue

            # 定位成功，存入有效列表
            valid_entries.append({
                'text':      text,                   # 识别字符
                'score':     score,                  # 置信度
                'bbox':      bbox,                   # 原始四点框
                'center_px': result['center_px'],    # 字符中心像素
                'point_base':  result['point_base'], # 基座系 3D 位置（射线-圆柱交点）
                'normal_base': result['normal_base'],# 基座系法向量（径向向外）
                'debug':       result['debug'],      # 调试信息
            })
            debug_rows.append({
                'text': text, 'score': score, 'center_px': center_px.tolist(),
                'raw_depth_mm': raw_center['depth_mm'],
                'raw_point_base': raw_center['point_base'],
                'processed_success': True,
                'processed_point_base': result['point_base'].tolist(),
                'processed_normal_base': result['normal_base'].tolist(),
                'fail_reason': '',
            })

        # --- 写调试追踪日志文件 ---
        self.write_debug_trace_txt(
            rgb_msg=rgb_msg, info_msg=info_msg, T_base_cam=T_base_cam,
            global_roi=global_roi, cyl_model=cyl_model, debug_rows=debug_rows,
            valid_count=len(valid_entries), total_count=len(detections),
        )

        # 至少需要 2 个有效字符才能生成轨迹（圆弧需要至少 2 个点确定方向和跨度）
        if len(valid_entries) < 2:
            response.message = 'Not enough valid chars after ray-cylinder intersection'
            self.save_debug_image(debug_image)
            return response

        # ==================================================================
        # 【阶段 C：轨迹生成】
        # ==================================================================

        # --- C0. 可选：按圆弧角度排序（默认按 OCR x 坐标）---
        if self.sort_by == 'arc_angle':
            valid_entries = self.sort_entries_by_arc_hint(valid_entries)

        # --- C1. 从字符 3D 位置 + 法向量估算截面圆模型 ---
        # 利用 surface_center_sign 处理内凹/外凸场景：
        #   inner/concave: center = point + radius * normal  （圆心在法向侧）
        #   outer/convex:  center = point - radius * normal  （圆心在法向对侧）
        # 对多个字符的中心候选点做中值滤波 + 均值融合，得到稳定圆心
        circle_model = self.estimate_circle_model(valid_entries)
        if circle_model is None:
            response.message = 'Failed to estimate arc circle model'
            self.save_debug_image(debug_image)
            return response

        # --- C2. 沿圆柱面构建每个字符的 6-DOF 姿态 ---
        # 将每个字符的 3D 点投影到截面圆上（强制落在圆弧上），
        # Z 轴 = 径向方向（指向/背离圆心），X 轴 = 圆弧切线方向
        valid_entries = self.build_arc_poses(valid_entries, circle_model)

        # --- C3. 轨迹密度化 ---
        if self.trajectory_mode == 'dense_arc':
            # 在字符点之间沿圆弧做等角度密度插值，
            # 采样步长 = arc_sample_step_m / radius（弧度）
            dense_poses = self.densify_arc_poses(valid_entries, circle_model)
            if not dense_poses:
                response.message = 'Dense arc generation failed'
                self.save_debug_image(debug_image)
                return response
            # dense_arc 模式下 texts 统一填 'ARC'（不再是逐个字符）
            response.poses.extend(dense_poses)
            response.texts = ['ARC'] * len(dense_poses)
        else:
            # 非密度模式：直接使用原始字符姿态
            for entry in valid_entries:
                response.poses.append(entry['pose'])
                response.texts.append(entry['text'])

        # --- C4. 法向偏移（工具长度补偿）---
        # 将轨迹中每个姿态沿其局部 Z 轴（法向）平移 trajectory_normal_offset_m
        # 默认 -0.035m，即向表面内偏移 35mm，补偿 TCP 与工具末端的距离
        if len(response.poses) > 0 and abs(self.trajectory_normal_offset_m) > 1e-9:
            response.poses = self.offset_poses_along_local_normal(
                response.poses, self.trajectory_normal_offset_m
            )

        # ==================================================================
        # 【可视化与发布】
        # ==================================================================

        # --- 构建 RViz MarkerArray ---
        marker_array = MarkerArray()
        # 先发一个 DELETEALL 清除上一帧的残留
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # 黄色球体：拟合截面圆的圆心 + 红色箭头：圆平面法向
        marker_array.markers.extend(self.build_circle_center_markers(circle_model))
        # 青色箭头：圆柱轴线方向
        marker_array.markers.extend(self.build_cylinder_axis_markers(cyl_model))

        if self.trajectory_mode == 'dense_arc':
            # 大绿球 + 蓝色法向箭头 + 文字标签：原始字符姿态（id 从 10000 起）
            for index, entry in enumerate(valid_entries):
                marker_array.markers.extend(self.build_markers(
                    index=index, text=entry['text'], pose_msg=entry['pose'],
                    ns_prefix='char_pose', id_offset=10000,
                    point_scale=0.018, axis_length=max(self.arc_tangent_length, 0.05),
                    text_scale=0.028, text_offset=0.04,
                    show_axis=True, show_text=True,
                ))
            # 小绿球：密度插值轨迹点（id 从 0 起，不显示法向和文字避免拥挤）
            for index, pose_msg in enumerate(response.poses):
                marker_array.markers.extend(self.build_markers(
                    index=index, text='ARC_{:03d}'.format(index),
                    pose_msg=pose_msg, ns_prefix='arc_dense', id_offset=0,
                    point_scale=0.006,
                    axis_length=max(self.arc_tangent_length * 0.5, 0.01),
                    text_scale=0.015, text_offset=0.02,
                    show_axis=False, show_text=False,
                ))

        # 发布 PoseArray 和 MarkerArray
        self.publish_pose_array(response.poses)
        self.marker_pub.publish(marker_array)

        # 保存带标注的调试图片到磁盘
        self.save_debug_image(debug_image)

        # ------------------------------------------------------------------
        # 最终成功判定：至少 1 个有效姿态 → True
        # ------------------------------------------------------------------
        response.success = len(response.poses) > 0
        response.message = (
            'Success: {} targets'.format(len(response.poses))
            if response.success
            else 'No valid poses'
        )
        return response

    def decode_bgr8(self, msg):
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            raise RuntimeError('RGB decode failed (encoding={}): {}'.format(msg.encoding, exc))

    def decode_depth16(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as exc:
            raise RuntimeError('Depth decode failed (encoding={}): {}'.format(msg.encoding, exc))

        if depth.dtype == np.uint16:
            return depth

        # 兼容部分驱动输出 32FC1（单位米）
        if depth.dtype == np.float32 or depth.dtype == np.float64:
            depth_mm = np.where(np.isfinite(depth), depth * 1000.0, 0.0)
            depth_mm = np.clip(depth_mm, 0.0, np.iinfo(np.uint16).max)
            return depth_mm.astype(np.uint16)

        raise RuntimeError('Unsupported depth dtype: {} (encoding={})'.format(depth.dtype, msg.encoding))

    def lookup_camera_transform(self, rgb_msg):
        stamp = rgb_msg.header.stamp
        camera_frame = rgb_msg.header.frame_id
        try:
            trans_stamped = self.tf_buffer.lookup_transform(self.base_frame, camera_frame, stamp, rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            try:
                trans_stamped = self.tf_buffer.lookup_transform(self.base_frame, camera_frame, rospy.Time(0.5), rospy.Duration(1.0))
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

    def compute_raw_center_point_from_depth(self, center_px, depth_img, fx, fy, cx, cy, T_base_cam, window_size = 5):
        ##字符在像素坐标系下的亚像素级中心点
        u_sub = float(center_px[0])
        v_sub = float(center_px[1])
        u = int(round(u_sub))
        v = int(round(v_sub))
        height, width = depth_img.shape
        if u < 0 or u >= width or v < 0 or v >= height:
            return {'depth_mm': None, 'point_base': None}
        
        half_w = window_size // 2
        u_min = max(0, u-half_w)
        u_max = min(width, u+half_w+1)
        v_min = max(0, v - half_w)
        v_max = min(height, v + half_w + 1)

        roi_depth = depth_img[v_min:v_max, u_min:u_max]
        valid_mask = (roi_depth >= self.depth_min_mm) & (roi_depth <= self.depth_max_mm)
        valid_depths = roi_depth[valid_mask]

        if len(valid_depths) == 0:
            return {'depth_mm': None, 'point_base': None}
        
        depth_mm = float(np.median(valid_depths))
        z = depth_mm / 1000.0

        x = (u_sub - cx) * z / fx
        y = (v_sub - cy) * z / fy
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

    # 1、重写圆柱拟合算法：基于全局先验轴向、先验半径，以及局部法向计算
    # def fit_cylinder_model(self, points_base, raw_centers_base):
    #     axis_dir = self.resolve_cylinder_axis_dir(raw_centers_base)
        
    #     ##计算全局点云的质心，并以此为中心进行局部平面拟合，提取局部法向作为径向方向的参考
    #     centroid = np.mean(points_base, axis=0)
    #     centered = points_base - centroid[None, :]

    #     # 对 ROI 点云进行平面拟合（局部近似平面），提取局部法向
    #     cov = centered.T @ centered / max(len(points_base) - 1, 1)
    #     eigvals, eigvecs = np.linalg.eigh(cov)
    #     local_normal = eigvecs[:, np.argmin(eigvals)]
        
    #     # 确保法向指向圆柱外部（远离圆心），依赖于你的 radial_reference_axis 设定
    #     if np.dot(local_normal, self.radial_reference_axis) < 0.0:
    #         local_normal = -local_normal
            
    #     # 利用先验名义半径，沿着局部法向向内回退，推导圆柱轴线经过的点 
    #     #可能需要更改 可能是+
    #     axis_point = centroid - self.nominal_radius_m * local_normal

    #     # 构造基底，维持后续接口的兼容性
    #     basis_u = self.project_reference_to_tangent_plane(axis_dir)
    #     basis_v = np.cross(axis_dir, basis_u)
    #     basis_v = self.safe_normalize(basis_v, np.array([0.0, 0.0, 1.0], dtype=np.float64))

    #     rospy.loginfo('基于局部法向与先验半径(%.4fm) 推算圆柱模型', self.nominal_radius_m)

    #     return {
    #         'axis_point': axis_point,
    #         'axis_dir': axis_dir,
    #         'radius': self.nominal_radius_m, # 半径
    #         'basis_u': basis_u,
    #         'basis_v': basis_v,
    #         'inlier_ratio': 1.0, # 伪造内点率，通过后续安全检查
    #     }
    
    def fit_cylinder_model(self, points_base, raw_centers_base):
        # 1. 锁定绝对物理先验轴向 (W 轴)
        axis_dir = self.resolve_cylinder_axis_dir(raw_centers_base)
        
        # 2. 构造垂直于轴向的 2D 投影平面正交基底 (U, V)
        W = axis_dir
        temp_vec = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        # 防止 W 刚好和 temp_vec 平行导致叉乘为 0
        if np.abs(np.dot(W, temp_vec)) > 0.9:
            temp_vec = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            
        U = self.safe_normalize(np.cross(temp_vec, W), np.array([1.0, 0.0, 0.0], dtype=np.float64))
        V = self.safe_normalize(np.cross(W, U), np.array([0.0, 1.0, 0.0], dtype=np.float64))

        # 3. 将 3D 散乱点云“拍扁”到 2D 投影平面
        centroid_3d = np.mean(points_base, axis=0)
        centered_3d = points_base - centroid_3d
        
        x_2d = centered_3d @ U
        y_2d = centered_3d @ V
        points_2d = np.column_stack((x_2d, y_2d))

        # 4. 在 2D 平面估算初始法向（仅用于提供寻心迭代的初始起点）
        cov_2d = points_2d.T @ points_2d / max(len(points_2d) - 1, 1)
        eigvals_2d, eigvecs_2d = np.linalg.eigh(cov_2d)
        normal_2d = eigvecs_2d[:, 0]  # 最小方差方向即为 2D 圆弧的法向

        # 将 3D 的参考朝向投影到 2D 平面
        ref_2d = np.array([np.dot(self.radial_reference_axis, U), np.dot(self.radial_reference_axis, V)])
        if np.dot(normal_2d, ref_2d) < 0.0:
            normal_2d = -normal_2d

        # 估算初始圆心 C0。因为 normal_2d 朝外，所以圆心在相反方向。
        # 注意：这里的 2D 原点是 3D 质心的投影。虽然质心不在圆弧面上，但作为优化算法的起点已经足够精确。
        c_guess = self.surface_center_sign * self.nominal_radius_m * normal_2d

        # 5. 非线性最小二乘法：在 2D 平面上精准锁定圆心
        def circle_residuals(c, pts, r):
            # 目标函数：让所有点到圆心 c 的距离，无限逼近于名义半径 r
            return np.linalg.norm(pts - c, axis=1) - r
        
        # 启动 Levenberg-Marquardt 优化器，寻找最优解 c_opt
        res = least_squares(circle_residuals, c_guess, args=(points_2d, self.nominal_radius_m))
        c_opt = res.x

        # 6. 升维还原：将算出的完美 2D 圆心，投射回 3D 物理空间
        axis_point = centroid_3d + c_opt[0] * U + c_opt[1] * V

        # 7. 构造原接口兼容的正交基底（保持下游轨迹生成代码的兼容性）
        basis_u = self.project_reference_to_tangent_plane(axis_dir)
        basis_v = np.cross(axis_dir, basis_u)
        basis_v = self.safe_normalize(basis_v, np.array([0.0, 0.0, 1.0], dtype=np.float64))

        rospy.loginfo('高精度 2D 投影寻心完成: 轴心偏移量(2D_norm)=%.4f mm', 
                      np.linalg.norm(c_opt - c_guess) * 1000.0)

        return {
            'axis_point': axis_point,
            'axis_dir': axis_dir,
            'radius': self.nominal_radius_m,
            'basis_u': basis_u,
            'basis_v': basis_v,
            'inlier_ratio': 1.0, 
        }

    # def resolve_cylinder_axis_dir(self, raw_centers_base):
    #     # 防御 1：如果只有一个点或没有点，两点才能成线，必须退回到固定先验
    #     if len(raw_centers_base) < 2:
    #         rospy.logwarn('字符中心点不足 2 个，无法拟合动态轴向，退回固定轴向')
    #         return self.cyl_axis_fixed_base.copy()

    #     # 将列表转换为 NumPy 矩阵 (N, 3)
    #     pts = np.asarray(raw_centers_base, dtype=np.float64)

    #     # 核心数学：PCA 拟合 3D 直线
    #     # 1. 算质心并去中心化
    #     centroid = np.mean(pts, axis=0)
    #     centered = pts - centroid
        
    #     # 2. 计算协方差矩阵并求特征值/特征向量
    #     cov = centered.T @ centered
    #     eigvals, eigvecs = np.linalg.eigh(cov)
        
    #     # 3. 提取最大特征值对应的特征向量（即点云分布最长的那条线的方向）
    #     axis_dir = eigvecs[:, np.argmax(eigvals)]

    #     # 防御 2：解决方向二义性（防止直线向量每次发生 180 度翻转）
    #     # 我们利用固定先验作为一个大致的参考方向，把算出来的轴向统一拨正
    #     if np.dot(axis_dir, self.cyl_axis_fixed_base) < 0.0:
    #         axis_dir = -axis_dir

    #     # 确保向量长度严格为 1
    #     axis_dir = self.safe_normalize(axis_dir, self.cyl_axis_fixed_base)

    #     rospy.loginfo('【实验】动态拟合字符连线得出轴向: [%.4f, %.4f, %.4f]', 
    #                   axis_dir[0], axis_dir[1], axis_dir[2])
    #     return axis_dir
    # 废弃 PCA，返回固定的物理轴向先验
    def resolve_cylinder_axis_dir(self, raw_centers_base):
        axis_dir = self.cyl_axis_fixed_base.copy()
        rospy.loginfo('直接采用固定先验轴向: [%.4f, %.4f, %.4f]', 
                      axis_dir[0], axis_dir[1], axis_dir[2])
        return axis_dir

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
        center_candidates = points + self.surface_center_sign * self.nominal_radius_m * normals

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

    def offset_poses_along_local_normal(self, poses, offset_m):
        """
        将轨迹中的每个 Pose 沿其自身局部法向（局部 Z 轴）平移指定距离。
        """
        shifted_poses = []
        for pose in poses:
            rotation = R.from_quat([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ])
            normal = rotation.apply(np.array([0.0, 0.0, 1.0], dtype=np.float64))

            shifted_pose = copy.deepcopy(pose)
            shifted_pose.position.x = float(pose.position.x + normal[0] * offset_m)
            shifted_pose.position.y = float(pose.position.y + normal[1] * offset_m)
            shifted_pose.position.z = float(pose.position.z + normal[2] * offset_m)
            shifted_poses.append(shifted_pose)

        return shifted_poses

    def publish_pose_array(self, poses):
        pose_array = PoseArray()
        pose_array.header.frame_id = self.base_frame
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses = [copy.deepcopy(pose) for pose in poses]
        self.pose_array_pub.publish(pose_array)

    def save_debug_image(self, debug_image):
        filename = '{}.jpg'.format(self.debug_image_prefix)
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
            lines.append('surface_mode: {}  center_sign: {:.1f}'.format(self.surface_mode, self.surface_center_sign))
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