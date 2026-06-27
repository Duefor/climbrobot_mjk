#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ================== 【关键修改 1/3】最顶部强制设置环境变量 ==================
import os
import sys

# 1. 强制预加载系统 libgomp (请先用 find /usr/lib -name "libgomp.so.1" 确认路径)
os.environ["LD_PRELOAD"] = "/usr/lib/aarch64-linux-gnu/libgomp.so.1"
# os.environ["LD_PRELOAD"] = "/usr/lib/x86_64-linux-gnu/libgomp.so.1"

# 2. 限制 OpenMP 线程数，双保险
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"

# 3. 强制预导入 sklearn (虽然这个脚本没直接用，但 paddleocr/albumentations 会间接调用)
try:
    import sklearn
    print(f"[DEBUG] 成功预导入 sklearn: {sklearn.__version__}")
except ImportError as e:
    print(f"[WARN] sklearn 预导入失败: {e}")
# ==============================================================================

# 接下来是原本的基础导入
import threading
import time

# 处理 ROS 路径
ros_path = '/opt/ros/noetic/lib/python3/dist-packages'
sys_path = '/usr/lib/python3/dist-packages'

if os.path.exists(ros_path) and ros_path not in sys.path:
    sys.path.append(ros_path)
if os.path.exists(sys_path) and sys_path not in sys.path:
    sys.path.append(sys_path)

# ================== 【关键修改 2/3】提前导入 PaddleOCR ==================
import cv2
import numpy as np
from paddleocr import PaddleOCR  # <--- PaddleOCR 移到这里，紧跟 cv2/numpy
import paddleocr
print(f"[DEBUG] 成功导入 paddleocr: {paddleocr.__version__}")
import paddle
print(paddle.device.get_device())
# =========================================================================

# 剩下的原本的导入
import open3d as o3d
import rospy
import tf2_ros
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, Pose, PoseArray
from sensor_msgs.msg import Image as SensorImage, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import splprep, splev

from mv3d_rgbd_ros.srv import DetectSteelStamp, DetectSteelStampResponse


def parse_bool_param(value, default=False):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    if value is None:
        return default
    return bool(value)


class SteelStampOcrSoloServer:
    """
    针对单个字符进行独立“局部碎片”提取、降噪与拟合的 OCR 服务：
    1. 原始深度的粗略提取与基座标转换 
    2. 参数化尺寸的小方块点云碎片提取 (如 21x21)
    3. 物理重心与全局切平面拟合 (取代复杂的 MLS，以平均与协方差直接锁定最优 3D 锚点)
    4. KD-Tree 锚定最近平滑点 (用户删减优化版)
    5. 强制法向朝向，姿态重构
    6. 三次 B 样条 (Cubic B-Spline) 轨迹连续平滑插值输出
    """
    def __init__(self):
        rospy.init_node('steel_stamp_ocr_solo_server_node')

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
        
        self.depth_min_mm = int(rospy.get_param('~depth_min_mm', 300))
        self.depth_max_mm = int(rospy.get_param('~depth_max_mm', 1000))

        # 新增扩展配置，21为默认ROI提取大小
        self.roi_window_size = int(rospy.get_param('~roi_window_size', 21))
        
        # 新增插值配置
        self.trajectory_mode = rospy.get_param('~trajectory_mode', 'dense_arc')
        self.arc_sample_step_m = float(rospy.get_param('~arc_sample_step_m', 0.004))
        self.max_arc_points = int(rospy.get_param('~max_arc_points', 500))
        # 新增法向偏移配置，默认 -0.048m（-48mm）向工件表面内偏移
        self.trajectory_normal_offset_m = float(rospy.get_param('~trajectory_normal_offset_m', -0.036))
        self.cylinder_diameter_m = float(rospy.get_param('~cylinder_diameter_m', 4.3))
        self.cylinder_radius_m = 0.5 * self.cylinder_diameter_m
        self.debug_image_prefix = rospy.get_param('~debug_image_prefix', 'cyl_solo_result')
        # True: point -> circle center. False: circle center -> point.
        self.pose_z_to_center = parse_bool_param(rospy.get_param('~pose_z_to_center', False), False)

        # 姿态切向参考向
        self.pose_reference_axis = np.asarray(rospy.get_param('~pose_reference_axis', [1.0, 0.0, 0.0]), dtype=np.float64)
        self.pose_reference_axis_backup = np.asarray(rospy.get_param('~pose_reference_axis_backup', [0.0, 1.0, 0.0]), dtype=np.float64)
        
        # 基座负方向，用于约束法向
        self.base_negative_axis = np.asarray(rospy.get_param('~base_negative_axis', [0.0, 0.0, -1.0]), dtype=np.float64)

        self.pose_reference_axis = self.safe_normalize(self.pose_reference_axis, np.array([1.0, 0.0, 0.0], dtype=np.float64))
        self.pose_reference_axis_backup = self.safe_normalize(self.pose_reference_axis_backup, np.array([0.0, 1.0, 0.0], dtype=np.float64))
        self.base_negative_axis = self.safe_normalize(self.base_negative_axis, np.array([0.0, 0.0, 1.0], dtype=np.float64))

        self.latest_data = {'rgb': None, 'depth': None, 'info': None}
        self.data_lock = threading.Lock()

        script_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_root = os.path.dirname(script_dir)
        self.save_dir = os.path.join(pkg_root, 'debug_images')
        os.makedirs(self.save_dir, exist_ok=True)

        # 加载 OCR 模型
        self.det_model_path = '/home/m/ws_moveit/PaddleOCR/inference/det_steel_wall_new'
        self.rec_model_path = '/home/m/ws_moveit/PaddleOCR/inference/steel_rec_model'
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
        self.marker_pub = rospy.Publisher('/debug/steel_markers_solo', MarkerArray, queue_size=10, latch=True)
        self.pose_array_pub = rospy.Publisher('/debug/steel_pose_array_solo', PoseArray, queue_size=1, latch=True)

        rospy.loginfo('独立局部极平滑与三次 B 样条 OCR 节点已启动, service=%s', self.service_name)
        rospy.loginfo('pose_z_to_center=%s', str(self.pose_z_to_center))
        rospy.loginfo('cylinder_diameter_m=%.4f cylinder_radius_m=%.4f',
                      self.cylinder_diameter_m, self.cylinder_radius_m)

    def sync_callback(self, rgb_msg, depth_msg, info_msg):
        with self.data_lock:
            self.latest_data['rgb'] = rgb_msg
            self.latest_data['depth'] = depth_msg
            self.latest_data['info'] = info_msg

    def safe_normalize(self, v, fallback):
        n = np.linalg.norm(v)
        if n < 1e-9:
            return fallback.copy()
        return v / n

    def handle_req(self, _req):
        # =====================================================================
        # handle_req — ROS 服务回调，是整个 OCR + 轨迹生成的核心流程。
        #
        # 整体管线：
        #   同步图像获取 → 新鲜度检查 → 图像解码 → PaddleOCR 检测+识别
        #   → 按 x 排序字符 → TF 查询相机→基座变换 → 遍历每个检测框：
        #       ① 提取中心像素原始深度
        #       ② 抠取 roi_window_size × roi_window_size 局部深度窗口
        #       ③ 深度范围过滤 (300~600mm)，无效时用窗口中位数替补
        #       ④ 针孔模型逆投影：像素 (u,v) + 深度 Z → 相机 3D → 基座 3D
        #       ⑤ Open3D 统计离群点去除 (SOR 滤波)
        #       ⑥ 点云重心 = 最优 3D 锚点
        #       ⑦ 协方差矩阵 PCA → 最小特征值对应法向量
        #       ⑧ Gram-Schmidt 正交化构建旋转矩阵 → 四元数姿态
        #   → 三次 B 样条平滑拟合 → 弧长等距重采样 → 密集轨迹
        #   → 沿各点局部法向偏移（工具长度补偿）
        #   → 发布 RViz 可视化 Marker + PoseArray
        #   → 返回 {success, message, texts[], poses[]}
        #
        # 参数:
        #   _req: DetectSteelStamp 服务请求（空请求，仅用于触发检测）
        #         前缀 _ 表示该参数在函数体内未被使用
        #
        # 返回:
        #   DetectSteelStampResponse:
        #     - success (bool):   是否至少生成了 1 个有效姿态
        #     - message (string): 人类可读的状态描述
        #     - texts   (string[]): 识别到的字符文本列表（如 ['A','3','8']）
        #     - poses   (Pose[]):   最终轨迹姿态列表（可能经过 B 样条插值+法向偏移）
        # =====================================================================

        # ------------------------------------------------------------------
        # 1. 初始化响应对象，悲观设 success=False（防御性编程）
        #    只有所有步骤全部通过才会改为 True，中途任何 return 都保持 False
        # ------------------------------------------------------------------
        response = DetectSteelStampResponse()
        response.success = False

        # ------------------------------------------------------------------
        # 2. 线程安全地读取最新同步数据
        #    sync_callback（运行在 ROS 回调线程）持续写入 self.latest_data
        #    handle_req（运行在 ROS 服务线程）在此读取
        #    threading.Lock() 保证互斥，防止读到写了一半的脏数据
        # ------------------------------------------------------------------
        with self.data_lock:
            rgb_msg   = self.latest_data['rgb']    # sensor_msgs/Image — RGB 彩色图像
            depth_msg = self.latest_data['depth']  # sensor_msgs/Image — 深度图 (uint16, 毫米)
            info_msg  = self.latest_data['info']   # sensor_msgs/CameraInfo — 相机内参

        # ------------------------------------------------------------------
        # 3. 数据就绪检查
        #    节点刚启动时 ApproximateTimeSynchronizer 可能还没收到第一组数据
        #    此时三个字段均为 None，直接返回失败让调用方稍后重试
        # ------------------------------------------------------------------
        if rgb_msg is None or depth_msg is None or info_msg is None:
            response.message = 'No synchronized RGB/Depth/CameraInfo received yet'
            return response

        # ------------------------------------------------------------------
        # 4. 数据新鲜度检查
        #    计算 当前ROS时间 - 图像时间戳 的差值（秒）
        #    > 1.0s 说明图像缓存过久，可能发生处理拥塞或相机断流
        #    选择”快速失败”而非等待，避免阻塞 ROS 服务调用方
        # ------------------------------------------------------------------
        latency = (rospy.Time.now() - rgb_msg.header.stamp).to_sec()
        if latency > 1.0:
            response.message = f'Data is too old (latency: {latency:.2f}s).'
            rospy.logerr(response.message)  # 输出 ERROR 级别日志到 rosout
            return response

        # ------------------------------------------------------------------
        # 5. ROS 图像消息 → OpenCV numpy 数组
        #    - rgb_arr:   (H, W, 3) uint8,  BGR 通道顺序（OpenCV/PaddleOCR 默认）
        #    - depth_arr: (H, W)   uint16, 单位毫米（passthrough=不解码，保留原始格式）
        #    如果深度图编码不是 uint16（某些相机驱动可能输出 uint32/float32），强制转换
        # ------------------------------------------------------------------
        try:
            rgb_arr   = self.bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding='bgr8')
            depth_arr = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            if depth_arr.dtype != np.uint16:
                depth_arr = depth_arr.astype(np.uint16, copy=False)
        except (CvBridgeError, Exception) as exc:
            response.message = f'Image parse error: {exc}'
            return response

        # ------------------------------------------------------------------
        # 6. PaddleOCR 检测 + 识别
        #    cls=False: 关闭角度分类器，钢印字符一般不会旋转 180°，省计算
        #    返回值格式（PaddleOCR 官方格式）:
        #      ocr_results = [
        #        [  # 第一张图片的检测结果
        #          [[[x1,y1],[x2,y2],[x3,y3],[x4,y4]], ['text', confidence]],
        #          [[[x1,y1],[x2,y2],[x3,y3],[x4,y4]], ['text', confidence]],
        #          ...
        #        ]
        #      ]
        #    我们只传入一张 RGB 图，所以取 ocr_results[0]
        # ------------------------------------------------------------------
        ocr_results = self.ocr.ocr(rgb_arr, cls=False)
        if not ocr_results or not ocr_results[0]:
            # ocr_results 可能为 None（异常）或 [](空列表，未检测到文字)
            response.message = 'OCR detected nothing'
            return response

        # ------------------------------------------------------------------
        # 7. 按 x 坐标从左到右排序检测框
        #    line[0] 是四点框 [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
        #    np.mean(..., axis=0) → 中心点 [cx, cy]
        #    [:, 0] → 取 x 坐标作为排序键
        #    排序后字符从左到右排列，对应钢印号的自然阅读顺序
        # ------------------------------------------------------------------
        detections = ocr_results[0]
        detections.sort(key=lambda item: np.mean(np.asarray(item[0], dtype=np.float32)[:, 0]))

        # ------------------------------------------------------------------
        # 8. TF 查询：获取 相机坐标系 → 机器人基座坐标系 的 4×4 齐次变换矩阵
        #    T_base_cam 将相机系下的 3D 点变换到 base_link 系:
        #      P_base = T_base_cam @ P_cam
        #    使用图像时间戳做精确同步查询（与图像帧一一对应）
        # ------------------------------------------------------------------
        T_base_cam = self.lookup_camera_transform(rgb_msg)
        if T_base_cam is None:
            response.message = 'TF lookup failed'
            return response

        # ------------------------------------------------------------------
        # 9. 解析相机投影矩阵 P（3×4 → 提取内参）
        #    P 矩阵布局（一维索引，行优先）:
        #      [0]=fx,  [1]=0,   [2]=cx,  [3]=Tx
        #      [4]=0,   [5]=fy,  [6]=cy,  [7]=Ty
        #      [8]=0,   [9]=0,   [10]=1,  [11]=0
        #    fx, fy: 焦距（像素单位），cx, cy: 光心坐标（像素）
        #    这是针孔相机模型的核心参数，用于将像素坐标反投影为 3D 射线
        # ------------------------------------------------------------------
        fx, fy = float(info_msg.P[0]), float(info_msg.P[5])
        cx, cy = float(info_msg.P[2]), float(info_msg.P[6])
        height, width = depth_arr.shape  # 图像尺寸，用于后续边界检查
        debug_image = rgb_arr.copy()

        # ------------------------------------------------------------------
        # 10. 初始化处理变量
        #     valid_entries: 累积所有成功处理的字符数据
        #       每个元素 = {'text', 'pose', 'point', 'normal'}
        #     R_base_cam: 旋转部分 (3×3)，t_base_cam: 平移部分 (3,)
        #       从齐次矩阵分离，避免后续逐点变换时重复索引
        #     half_win: 局部窗口半宽（roi_window_size // 2），默认 21→10
        # ------------------------------------------------------------------
        raw_entries = []

        R_base_cam = T_base_cam[:3, :3]  # 左上 3×3 旋转矩阵
        t_base_cam = T_base_cam[:3, 3]   # 第 4 列前 3 行 = 平移向量

        half_win = self.roi_window_size // 2  # 整数除法，如 21//2 = 10

        # ==================================================================
        # 主循环：遍历每个 OCR 检测到的字符，独立计算其 3D 姿态
        # ==================================================================
        for line in detections:
            # ----------------------------------------------------------
            # 10a. 解析检测框和识别文本
            #      bbox shape=(4,2): 四个角点的像素坐标 [[x1,y1],...,[x4,y4]]
            #      text: 识别出的字符文本，如 'A', '3', '8'
            # ----------------------------------------------------------
            bbox = np.asarray(line[0], dtype=np.float32)  # 检测框四点坐标
            text = line[1][0]                              # 识别的文本内容
            score = float(line[1][1])

            # ----------------------------------------------------------
            # 10b. 计算包围框的几何中心（像素坐标）
            #      np.mean(bbox, axis=0): 对四个角点取平均 → [cx, cy]
            #      round()→int: 四舍五入到最近整数像素
            #      注意：u=列(x), v=行(y)，与 numpy 索引 [v, u] 一致
            # ----------------------------------------------------------
            center_px = np.mean(bbox, axis=0)
            u_center = int(round(center_px[0]))  # 中心列坐标
            v_center = int(round(center_px[1]))  # 中心行坐标
            self.draw_debug_overlay(debug_image, bbox, center_px, text, score, False)

            # ----------------------------------------------------------
            # 10c. 边界检查
            #      如果中心像素超出图像范围（检测框部分在图像边缘外），
            #      则无法可靠提取局部深度窗口，直接跳过该字符
            # ----------------------------------------------------------
            if u_center < 0 or u_center >= width or v_center < 0 or v_center >= height:
                continue

            # ==========================================================
            # 步骤 1：提取中心像素的原始深度值（毫米 → 米）
            #     depth_arr 索引顺序为 [行(v), 列(u)]
            #     uint16 → float，保留原始精度便于后续计算
            # ==========================================================
            raw_depth = float(depth_arr[v_center, u_center])

            # ==========================================================
            # 步骤 2：抠取局部深度窗口并反投影为 3D 点云
            # ==========================================================

            # --- 2a. 计算局部窗口的像素范围 ---
            # max(0, ...): 防止左/上边界越界（Python 负索引会回绕末尾！）
            # min(width/height, ...): 防止右/下边界越界
            # +1: Python 切片 [start:end) 是左闭右开，+1 确保包含 end 像素
            u_min, u_max = max(0, u_center - half_win), min(width,  u_center + half_win + 1)
            v_min, v_max = max(0, v_center - half_win), min(height, v_center + half_win + 1)
            roi_depth = depth_arr[v_min:v_max, u_min:u_max]  # 局部深度窗口 (H_roi, W_roi)

            # --- 2b. 创建深度有效性掩码 ---
            # 只有深度值在 [depth_min_mm, depth_max_mm] 范围内的像素才有效
            # 过滤掉：背景(0或很大)、遮挡物(<300mm)、远处无关区域(>600mm)
            # valid_mask 是与 roi_depth 同 shape 的 bool 数组
            valid_mask = (roi_depth >= self.depth_min_mm) & (roi_depth <= self.depth_max_mm)

            # --- 2c. 中心深度无效时的中值替补策略 ---
            # 如果中心像素深度超出范围或为 0（反光/遮挡/超量程）:
            #   从有效窗口中取中位数替代
            #   中位数比均值更鲁棒——不受个别离群噪声像素影响
            if raw_depth < self.depth_min_mm or raw_depth > self.depth_max_mm or raw_depth == 0:
                valid_depths = roi_depth[valid_mask]   # 布尔索引提取所有有效深度
                if len(valid_depths) == 0:
                    continue                           # 整个窗口都无效，放弃该字符
                raw_depth = float(np.median(valid_depths))

            # --- 2d. 构建像素坐标网格 ---
            # np.meshgrid 生成每个像素的 (u, v) 坐标矩阵
            #   例如 u_min=100, u_max=105 → 产生 5 列 [100,101,102,103,104]
            # 然后用 valid_mask 过滤，只保留有效像素的坐标和深度
            u_grid, v_grid = np.meshgrid(np.arange(u_min, u_max), np.arange(v_min, v_max))
            u_valid = u_grid[valid_mask].astype(np.float64)          # 有效像素的 u 坐标
            v_valid = v_grid[valid_mask].astype(np.float64)          # 有效像素的 v 坐标
            z_valid = roi_depth[valid_mask].astype(np.float64) / 1000.0  # 毫米→米

            # --- 2e. 最少点数检查 ---
            # 少于 10 个有效点 → 点云太稀疏，不足以可靠拟合平面
            # 对于默认 21×21=441 像素窗口，10 点意味着仅约 2.3% 有效率
            if len(z_valid) < 10:
                continue

            # --- 2f. 针孔相机模型逆投影：像素座标 → 相机 3D 坐标 ---
            # 标准针孔模型逆投影公式:
            #   X_cam = (u - cx) * Z / fx
            #   Y_cam = (v - cy) * Z / fy
            #   Z_cam = Z
            # 向量化操作：一次性对所有有效像素计算，避免 Python 循环
            x_valid = (u_valid - cx) * z_valid / fx
            y_valid = (v_valid - cy) * z_valid / fy
            points_cam = np.stack((x_valid, y_valid, z_valid), axis=-1)  # (N,3)

            # --- 2g. 坐标变换：相机系 → 机器人基座系 (base_link) ---
            # R_base_cam @ points_cam.T: (3×3) @ (3×N) = (3×N) 旋转
            # .T: 转置回 (N, 3)
            # + t_base_cam[None, :]: 广播加法，(N,3) + (1,3) = (N,3) 平移
            points_base = (R_base_cam @ points_cam.T).T + t_base_cam[None, :]

            # ==========================================================
            # 步骤 3-5：去噪 → 重心锚点 → PCA 法向量拟合
            # ==========================================================

            # --- 3. Open3D 统计离群点移除 (Statistical Outlier Removal, SOR) ---
            # 对每个点，计算它到最近 nb_neighbors=15 个邻居的平均距离
            # 如果该距离 > 全局均值 + std_ratio×全局标准差 → 判定为噪声并剔除
            # 这对钢件表面的反光飞点、深度跳变像素非常有效
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_base)       # numpy→Open3D (零拷贝)
            pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=15, std_ratio=1.0)
            if len(pcd.points) < 10:
                continue                                                # 去噪后点数不足，放弃
            filtered_points = np.asarray(pcd.points)                    # Open3D→numpy

            # --- 4. 物理重心 = 最优 3D 锚点 ---
            # 对于 21×21 像素的微小窗口（在 0.5m 距离约 7×7mm），
            # 表面几乎平坦，重心即是对该区域 3D 位置的最优估计
            # 这种方法比 MLS（移动最小二乘）简单但在此场景同样有效
            best_point = np.mean(filtered_points, axis=0)  # (3,) — 平均 x,y,z

            # --- 5. 协方差矩阵 PCA → 曲面法向量 ---
            # 原理：协方差矩阵的特征向量 = 数据的主方向
            #   最大特征值 → 方差最大方向（切平面内，数据展开最广）
            #   最小特征值 → 方差最小方向（数据变化最小的方向 = 法线方向）
            centered = filtered_points - best_point          # 中心化 (N,3) - (3,) → (N,3)
            cov = centered.T @ centered                      # 协方差矩阵 (3,N)@(N,3) = (3,3)
            eigvals, eigvecs = np.linalg.eigh(cov)           # 对称矩阵特征分解（升序排列）
            best_normal = eigvecs[:, 0]                      # 最小特征值对应的特征向量 = 法向量

            # 暂存字符中心点；最终法向在所有字符点估计出圆柱中心后统一计算。
            raw_entries.append({
                'text':   text,        # 识别的字符文本
                'bbox':   bbox,
                'center_px': center_px,
                'score':  score,
                'point':  best_point,  # 3D 锚点, (3,) numpy array (base_link 系)
                'local_normal': best_normal, # 仅作为圆柱中心估计失败时的后备方向
            })

        valid_entries = self.build_entries_with_center_normals(raw_entries)
        for entry in valid_entries:
            self.draw_debug_overlay(
                debug_image,
                entry['bbox'],
                entry['center_px'],
                entry['text'],
                entry['score'],
                True,
            )

        # ==================================================================
        # 11. 收集成功识别的文本列表
        #     即使后续轨迹生成失败，调用方也能获取检测到的字符
        # ==================================================================
        response.texts = [e['text'] for e in valid_entries]

        # ==================================================================
        # 12. 轨迹生成：三次 B 样条插值 或 直接使用原始姿态
        #     - dense_arc 模式 + ≥2 个有效点: B 样条拟合 → 弧长等距重采样
        #     - 其他情况: 直接使用原始检测姿态（单个字符或非插值模式）
        # ==================================================================
        final_poses = []
        used_interpolation = False
        if self.trajectory_mode == 'dense_arc' and len(valid_entries) >= 2:
            # 三次 B 样条平滑拟合 + 等距重采样 → 密集轨迹
            # 例如 5 个检测点 → B 样条拟合 → 沿弧长每 4mm 采样 → ~200 个轨迹点
            final_poses = self.generate_bspline_trajectory(valid_entries)
            used_interpolation = True
        else:
            # 不满足插值条件：直接取原始姿态作为轨迹
            final_poses = [e['pose'] for e in valid_entries]

        # ==================================================================
        # 13. 法向量偏移（工具长度补偿）
        #     对插值轨迹的每个姿态，沿其局部 Z 轴（拟合的曲面法向）平移
        #     默认 -0.048m（-48mm），即向工件表面内偏移约 5cm
        #     为什么需要？轨迹点位于工件表面，但机器人 TCP（工具中心点）
        #     在工具末端，需后退一个工具长度使 TCP 接触到工件表面
        #     条件：仅对插值结果做偏移（原始检测点不做）
        # ==================================================================
        if used_interpolation and len(final_poses) > 0 and abs(self.trajectory_normal_offset_m) > 1e-9:
            final_poses = self.offset_poses_along_local_normal(final_poses, self.trajectory_normal_offset_m)

        # ==================================================================
        # 14. 组装响应
        # ==================================================================
        response.poses = final_poses

        # ------------------------------------------------------------------
        # 15. 发布调试可视化数据（供 RViz 显示）
        #     - publish_markers:     绿色球体(锚点) + 蓝色箭头(法向) + 文字标签 + 紫色连线(轨迹)
        #     - publish_pose_array:  发布 PoseArray 到 /debug/steel_pose_array_solo
        #                            下游节点（运动规划/轨迹跟随）可直接订阅
        # ------------------------------------------------------------------
        self.publish_markers(valid_entries, final_poses)
        self.publish_pose_array(final_poses)
        self.save_debug_image(debug_image)

        # ------------------------------------------------------------------
        # 16. 设置最终响应状态
        #     success: 至少 1 个有效姿态 → True，否则 False
        #     message: 包含数量信息，便于日志分析和问题排查
        # ------------------------------------------------------------------
        response.success = len(response.poses) > 0
        response.message = (
            f'Success: {len(response.poses)} calculated poses'
            if response.success
            else 'No valid poses'
        )
        return response

    def build_entries_with_center_normals(self, raw_entries):
        """
        用所有字符中心点估计圆柱截面圆心，再用 point -> center 作为朝向壁面内部的法向。
        局部 PCA 法向只作为点数不足或圆心估计失败时的后备方向。
        """
        if not raw_entries:
            return []

        points = np.asarray([entry['point'] for entry in raw_entries], dtype=np.float64)
        center = self.estimate_cylinder_center_from_points(points)
        if center is None:
            rospy.logwarn('Cylinder center estimation failed; falling back to local PCA normals')
        else:
            rospy.loginfo(
                'Estimated cylinder center for solo normals: [%.4f, %.4f, %.4f]',
                center[0], center[1], center[2],
            )

        valid_entries = []
        previous_normal = None
        raw_radius_values = []
        projection_offsets = []
        for entry in raw_entries:
            raw_point = entry['point']
            if center is not None:
                radial_out = raw_point - center
                raw_radius = np.linalg.norm(radial_out)
                raw_radius_values.append(raw_radius)

                if raw_radius < 1e-9:
                    radial_out = self.safe_normalize(entry['local_normal'], self.base_negative_axis)
                else:
                    radial_out = radial_out / raw_radius

                projected_point = center + self.cylinder_radius_m * radial_out
                projection_offsets.append(np.linalg.norm(projected_point - raw_point))

                radial_to_center = center - projected_point
                normal = radial_to_center if self.pose_z_to_center else -radial_to_center
                normal = self.safe_normalize(normal, entry['local_normal'])
            else:
                projected_point = raw_point
                normal = entry['local_normal']
                if np.dot(normal, self.base_negative_axis) < 0:
                    normal = -normal
                normal = self.safe_normalize(normal, self.base_negative_axis)

            if previous_normal is not None and np.dot(normal, previous_normal) < 0.0:
                normal = -normal
            previous_normal = normal

            pose = self.pose_from_oriented_normal(projected_point, normal)
            valid_entries.append({
                'text': entry['text'],
                'bbox': entry['bbox'],
                'center_px': entry['center_px'],
                'score': entry['score'],
                'pose': pose,
                'raw_point': raw_point,
                'point': projected_point,
                'normal': normal,
            })

        if raw_radius_values:
            raw_radius_values = np.asarray(raw_radius_values, dtype=np.float64)
            projection_offsets = np.asarray(projection_offsets, dtype=np.float64)
            rospy.loginfo(
                'Solo fixed-radius projection: target_radius=%.4f raw_radius[min/mean/max]=[%.4f %.4f %.4f] max_projection_offset=%.4f',
                self.cylinder_radius_m,
                float(np.min(raw_radius_values)),
                float(np.mean(raw_radius_values)),
                float(np.max(raw_radius_values)),
                float(np.max(projection_offsets)) if len(projection_offsets) > 0 else 0.0,
            )

        return valid_entries

    def estimate_cylinder_center_from_points(self, points):
        if len(points) < 3:
            return None

        plane_origin = np.mean(points, axis=0)
        centered = points - plane_origin[None, :]
        cov = centered.T @ centered / max(len(points) - 1, 1)
        eigvals, eigvecs = np.linalg.eigh(cov)
        plane_normal = eigvecs[:, np.argmin(eigvals)]
        plane_normal = self.safe_normalize(plane_normal, self.base_negative_axis)

        basis_u = eigvecs[:, np.argmax(eigvals)]
        basis_u = basis_u - np.dot(basis_u, plane_normal) * plane_normal
        basis_u = self.safe_normalize(basis_u, self.pose_reference_axis)
        basis_v = np.cross(plane_normal, basis_u)
        basis_v = self.safe_normalize(basis_v, self.pose_reference_axis_backup)

        x = centered @ basis_u
        y = centered @ basis_v
        center_2d = self.fit_circle_center_2d(x, y)
        if center_2d is None:
            return None

        fixed_center_2d = self.refine_circle_center_with_fixed_radius_2d(
            x, y, center_2d, self.cylinder_radius_m)
        free_radius = np.sqrt(np.square(x - center_2d[0]) + np.square(y - center_2d[1]))
        fixed_radius = np.sqrt(np.square(x - fixed_center_2d[0]) + np.square(y - fixed_center_2d[1]))
        rospy.loginfo(
            'Solo circle fit: free_radius_mean=%.4f fixed_radius_target=%.4f fixed_radius[min/mean/max]=[%.4f %.4f %.4f]',
            float(np.mean(free_radius)),
            self.cylinder_radius_m,
            float(np.min(fixed_radius)),
            float(np.mean(fixed_radius)),
            float(np.max(fixed_radius)),
        )

        return plane_origin + fixed_center_2d[0] * basis_u + fixed_center_2d[1] * basis_v

    def fit_circle_center_2d(self, x, y):
        if len(x) < 3:
            return None

        a = np.column_stack((2.0 * x, 2.0 * y, np.ones_like(x)))
        b = x * x + y * y
        try:
            solution, _, rank, _ = np.linalg.lstsq(a, b, rcond=None)
        except np.linalg.LinAlgError:
            return None
        if rank < 3:
            return None

        return np.array([solution[0], solution[1]], dtype=np.float64)

    def refine_circle_center_with_fixed_radius_2d(self, x, y, initial_center, radius):
        center = np.asarray(initial_center, dtype=np.float64).copy()
        points = np.column_stack((x, y)).astype(np.float64)
        radius = float(radius)
        if len(points) < 2 or radius <= 1e-9:
            return center

        for _ in range(20):
            diff = center[None, :] - points
            distances = np.linalg.norm(diff, axis=1)
            valid = distances > 1e-9
            if np.count_nonzero(valid) < 2:
                break

            residual = distances[valid] - radius
            jacobian = diff[valid] / distances[valid, None]
            try:
                delta, _, _, _ = np.linalg.lstsq(jacobian, -residual, rcond=None)
            except np.linalg.LinAlgError:
                break

            center += delta
            if np.linalg.norm(delta) < 1e-6:
                break

        return center

    def generate_bspline_trajectory(self, valid_entries):
        """
        基于三次 B 样条 (Cubic B-Spline) 分别对 3D 锚点和 3D 法向量进行平滑拟合，
        并且生成定步长（arc_sample_step_m）的密集插值路径。
        """
        points = np.array([e['point'] for e in valid_entries])
        normals = np.array([e['normal'] for e in valid_entries])
        
        # 自动缩减阶数：如果点数不足 4 个，则降阶为 2 (二次) 或 1 (线性)
        k_degree = min(3, len(points) - 1)
        
        # 计算 3D 轨迹 B 样条，基于累积弦长进行参数化 (s=0 强制穿过特征点)
        tck_pts, u_pts = splprep(points.T, s=0, k=k_degree)
        
        # 在 u ∈ [0, 1] 做极密均匀采样，为下一步沿曲线距离步进抽样打基础
        u_fine = np.linspace(0, 1, 1000)
        fine_pts = np.vstack(splev(u_fine, tck_pts)).T
        
        # 计算整条平滑曲线的物理总长度
        diffs = np.diff(fine_pts, axis=0)
        dists = np.linalg.norm(diffs, axis=1)
        cum_dists = np.insert(np.cumsum(dists), 0, 0)
        total_len = cum_dists[-1]
        
        # 依照 arc_sample_step_m 等距切割插值轨迹点
        num_samples = max(2, int(np.ceil(total_len / self.arc_sample_step_m)) + 1)
        num_samples = min(num_samples, self.max_arc_points)  # 约束上限
        
        target_dists = np.linspace(0, total_len, num_samples)
        
        # 通过一维插值，将距离反向映射到 u 参数里
        u_resampled = np.interp(target_dists, cum_dists, u_fine)
        
        # 解析出平滑后的实际路径点阵
        smooth_points = np.vstack(splev(u_resampled, tck_pts)).T
        
        # 用相同方式计算目标法向。强制使其映射到与原点同一种分布。
        tck_norms, _ = splprep(normals.T, u=u_pts, s=0, k=k_degree)
        smooth_normals = np.vstack(splev(u_resampled, tck_norms)).T
        
        # 因为平滑后的法向模长可能会畸变，需要重新归一化
        norms_mag = np.linalg.norm(smooth_normals, axis=1, keepdims=True)
        norms_mag[norms_mag == 0] = 1.0
        smooth_normals = smooth_normals / norms_mag
        
        # 合成密集的机器人打磨轨线姿态
        dense_poses = []
        for pt, nrm in zip(smooth_points, smooth_normals):
            dense_poses.append(self.pose_from_oriented_normal(pt, nrm))
            
        return dense_poses

    def pose_from_oriented_normal(self, point, normal):
        """
        使用已经定好方向的法向构造姿态，不再按固定 base 轴翻转。
        """
        Z = self.safe_normalize(normal, self.base_negative_axis)
        X_ref = self.pose_reference_axis
        X_tangent = X_ref - np.dot(X_ref, Z) * Z

        if np.linalg.norm(X_tangent) < 1e-6:
            X_ref = self.pose_reference_axis_backup
            X_tangent = X_ref - np.dot(X_ref, Z) * Z

        X = X_tangent / np.linalg.norm(X_tangent)
        Y = np.cross(Z, X)
        Y = Y / np.linalg.norm(Y)
        X = np.cross(Y, Z)

        rot_mat = np.column_stack((X, Y, Z))
        quat = R.from_matrix(rot_mat).as_quat()

        pose = Pose()
        pose.position.x = float(point[0])
        pose.position.y = float(point[1])
        pose.position.z = float(point[2])
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        return pose

    def normal_to_pose(self, point, normal):
        """
        利用正交矩阵与四元数重构无干涉法向姿态
        """
        # 强制法向朝向: 与 base_negative_axis 保持同向
        if np.dot(normal, self.base_negative_axis) < 0:
            normal = -normal
            
        Z = normal
        X_ref = self.pose_reference_axis
        X_tangent = X_ref - np.dot(X_ref, Z) * Z
        
        # 奇异情况后备处理
        if np.linalg.norm(X_tangent) < 1e-6:
            X_ref = self.pose_reference_axis_backup
            X_tangent = X_ref - np.dot(X_ref, Z) * Z
            
        X = X_tangent / np.linalg.norm(X_tangent)
        Y = np.cross(Z, X)
        Y = Y / np.linalg.norm(Y)
        X = np.cross(Y, Z) # 确保绝对正交化
        
        rot_mat = np.column_stack((X, Y, Z))
        quat = R.from_matrix(rot_mat).as_quat()
        
        pose = Pose()
        pose.position.x = float(point[0])
        pose.position.y = float(point[1])
        pose.position.z = float(point[2])
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        return pose

    def offset_poses_along_local_normal(self, poses, offset_m):
        """
        将轨迹中的每个 Pose 沿其自身局部法向（局部 Z 轴）平移指定距离。
        """
        shifted_poses = []
        for pose in poses:
            rot = R.from_quat([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ])
            normal = rot.apply([0.0, 0.0, 1.0])

            shifted = Pose()
            shifted.position.x = float(pose.position.x + normal[0] * offset_m)
            shifted.position.y = float(pose.position.y + normal[1] * offset_m)
            shifted.position.z = float(pose.position.z + normal[2] * offset_m)
            shifted.orientation = pose.orientation
            shifted_poses.append(shifted)

        return shifted_poses

    def draw_debug_overlay(self, image, bbox, center_px, text, score, success):
        box_int = np.round(bbox).astype(np.int32)
        color = (0, 255, 0) if success else (0, 165, 255)
        cv2.polylines(image, [box_int], isClosed=True, color=color, thickness=2)

        cx, cy = int(round(center_px[0])), int(round(center_px[1]))
        cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)

        status = 'OK' if success else 'DET'
        label = '{} {:.2f} {}'.format(text, score, status)
        text_x = int(box_int[0][0])
        text_y = max(20, int(box_int[0][1]) - 8)
        cv2.putText(
            image,
            label,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

    def save_debug_image(self, debug_image):
        filename = '{}_{}.jpg'.format(self.debug_image_prefix, int(time.time() * 1000))
        path = os.path.join(self.save_dir, filename)
        cv2.imwrite(path, debug_image)
        rospy.loginfo('Solo OCR debug image saved: %s', path)
    
    def lookup_camera_transform(self, rgb_msg):
        stamp = rgb_msg.header.stamp
        camera_frame = rgb_msg.header.frame_id
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, 
                camera_frame, 
                stamp, 
                rospy.Duration(1.0)
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as exc:
            rospy.logerr(f'严格时间戳 TF 查询失败 : {exc}')
            return None
        
        q, t = trans.transform.rotation, trans.transform.translation
        mat = np.eye(4)
        mat[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        mat[:3, 3] = [t.x, t.y, t.z]
        return mat

    def publish_markers(self, valid_entries, final_poses):
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # 1. 绘制字符本身的关键点及文字
        for index, entry in enumerate(valid_entries):
            pose = entry['pose']
            text = entry['text']
            
            # Point Marker
            pm = Marker()
            pm.header.frame_id = self.base_frame
            pm.header.stamp = rospy.Time.now()
            pm.ns = 'solo_points'
            pm.id = index * 3
            pm.type = Marker.SPHERE
            pm.action = Marker.ADD
            pm.pose = pose
            pm.scale.x = pm.scale.y = pm.scale.z = 0.015
            pm.color.g = 1.0; pm.color.a = 1.0
            marker_array.markers.append(pm)
            
            # Axis Marker (Z 法向箭头)
            am = Marker()
            am.header.frame_id = self.base_frame
            am.header.stamp = rospy.Time.now()
            am.ns = 'solo_axis'
            am.id = index * 3 + 1
            am.type = Marker.ARROW
            am.action = Marker.ADD
            am.scale.x = 0.005; am.scale.y = 0.01; am.scale.z = 0.01
            am.color.b = 1.0; am.color.a = 1.0
            
            rot = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            direction = rot.apply([0, 0, 1])
            am.points = [
                Point(x=pose.position.x, y=pose.position.y, z=pose.position.z),
                Point(x=pose.position.x + direction[0]*0.05,
                      y=pose.position.y + direction[1]*0.05,
                      z=pose.position.z + direction[2]*0.05)
            ]
            marker_array.markers.append(am)
            
            # Text Marker
            tm = Marker()
            tm.header.frame_id = self.base_frame
            tm.header.stamp = rospy.Time.now()
            tm.ns = 'solo_text'
            tm.id = index * 3 + 2
            tm.type = Marker.TEXT_VIEW_FACING
            tm.action = Marker.ADD
            tm.pose.position.x = pose.position.x
            tm.pose.position.y = pose.position.y
            tm.pose.position.z = pose.position.z + 0.03
            tm.pose.orientation.w = 1.0
            tm.text = f"[{text}]"
            tm.scale.z = 0.025
            tm.color.r = 1.0; tm.color.g = 1.0; tm.color.b = 1.0; tm.color.a = 1.0
            marker_array.markers.append(tm)
            
        # 2. 如果是密集模式且点足够，绘制一根明显的 B-Spline 线段验证插值效果
        if self.trajectory_mode == 'dense_arc' and len(final_poses) > 1:
            lm = Marker()
            lm.header.frame_id = self.base_frame
            lm.header.stamp = rospy.Time.now()
            lm.ns = 'solo_bspline_trajectory'
            lm.id = 9999
            lm.type = Marker.LINE_STRIP
            lm.action = Marker.ADD
            lm.scale.x = 0.003  # 线宽
            lm.color.r = 1.0; lm.color.g = 0.0; lm.color.b = 1.0; lm.color.a = 1.0  # 紫色连线
            for p in final_poses:
                lm.points.append(Point(x=p.position.x, y=p.position.y, z=p.position.z))
            marker_array.markers.append(lm)

        self.marker_pub.publish(marker_array)

    def publish_pose_array(self, poses):
        pa = PoseArray()
        pa.header.frame_id = self.base_frame
        pa.header.stamp = rospy.Time.now()
        pa.poses = poses
        self.pose_array_pub.publish(pa)

if __name__ == '__main__':
    try:
        server = SteelStampOcrSoloServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
