#!/home/barry/workspace/ws_moveit/venv_ocr/bin/python3
# -*- coding: utf-8 -*-


import sys
import os

# ==============================================================================
# 关键修改：让虚拟环境"借用"系统的 ROS 库
# ==============================================================================
# 1. 路径 A: ROS 核心库 (rospy, cv_bridge, message 等)
ros_path = '/opt/ros/noetic/lib/python3/dist-packages'

# 2. 路径 B: 系统 Python 库 (rospkg, catkin_pkg 等基础工具)
# Ubuntu 上 apt 安装的 python 包通常在这里
sys_path = '/usr/lib/python3/dist-packages'

if os.path.exists(ros_path) and ros_path not in sys.path:
    sys.path.append(ros_path)

if os.path.exists(sys_path) and sys_path not in sys.path:
    sys.path.append(sys_path)

import threading
import copy
import numpy as np
import cv2
import rospy
import message_filters  # 用于时间同步
import tf2_ros
import tf2_geometry_msgs
import open3d as o3d     # 点云处理核心库
from scipy.spatial.transform import Rotation as R

# ROS 消息类型
from sensor_msgs.msg import Image as SensorImage, CameraInfo
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped

from visualization_msgs.msg import Marker, MarkerArray

# 自定义服务
from mv3d_rgbd_ros.srv import DetectSteelStamp, DetectSteelStampResponse
from paddleocr import PaddleOCR , draw_ocr

class SteelStamp3DServer:
    def __init__(self):
        rospy.init_node("steel_stamp_server_node")
        
        # 1.1 TF 监听器 (用于查询 相机->基座 的变换)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 1.2 PaddleOCR 初始化
        # 检测模型和识别模型路径
        self.det_model_path = "/home/barry/workspace/ws_moveit/PaddleOCR/inference/det_steel_best_v2/"
        self.rec_model_path = "/home/barry/workspace/ws_moveit/PaddleOCR/inference/rec_steel_best_v2/"
        # self.rec_char_dict_path='/home/barry/workspace/ws_moveit/PaddleOCR/train_data/steel_rec_dataset/my_steel_dict.txt'

        self.ocr = PaddleOCR(
            use_angle_cls = True,  # 应对相机旋转
            lang='en', use_gpu=True,
            det_model_dir=self.det_model_path,
            rec_model_dir=self.rec_model_path,
            # rec_char_dict_path=self.rec_char_dict_path,
            rec = True,
            det_algorithm='DB',
            det_limit_side_len=1600,
            det_db_thresh=0.6,
            det_db_box_thresh=0.6,
            det_db_unclip_ratio=1.6,
            show_log=False
        )
        
        # ==========================================
        # 2. 数据通信初始化
        # ==========================================
        # 2.1 数据缓存区
        self.latest_data = {
            "rgb": None,
            "depth": None,
            "info": None,
            "timestamp": None
        }
        self.data_lock = threading.Lock()
        
        # 2.2 多话题同步订阅
        # 同时订阅 RGB、深度图、相机内参
        # 注意：这里的话题名要和 camera_image_pub.py 发布的一致
        rgb_sub = message_filters.Subscriber("/camera/rgb/image_raw", SensorImage)
        depth_sub = message_filters.Subscriber("/camera/depth/image_raw", SensorImage)
        info_sub = message_filters.Subscriber("/camera/rgb/camera_info", CameraInfo)
        
        # 使用 ApproximateTimeSynchronizer (允许微小的时间误差，比 TimeSynchronizer 更稳定)
        # 队列长度 10，误差允许 0.1秒
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 10, 0.1)
        self.sync.registerCallback(self.sync_callback)
        
        # 2.3 启动服务
        self.srv = rospy.Service('get_steel_stamp_location', DetectSteelStamp, self.handle_req)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        pkg_root = os.path.dirname(script_dir)
        self.save_dir = os.path.join(pkg_root, "debug_images")

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        else:
            rospy.loginfo(f"{self.save_dir}")

        self.marker_pub = rospy.Publisher('/debug/steel_markers', MarkerArray, queue_size=10)

        rospy.loginfo(" [3D钢印检测] 服务已就绪！等待机械臂调用...")

    # 同步到一组三元组消息时，锁将它们存入缓存，服务被调用时可以立即取到最新一帧同步数据
    def sync_callback(self, rgb_msg, depth_msg, info_msg):
        """
        回调函数：只负责接收同步好的数据，存入缓存
        """
        with self.data_lock:
            self.latest_data["rgb"] = rgb_msg
            self.latest_data["depth"] = depth_msg
            self.latest_data["info"] = info_msg
            self.latest_data["timestamp"] = rgb_msg.header.stamp

    def handle_req(self, req):
        """
        服务处理主逻辑：收到请求 -> OCR -> 3D计算 -> 返回Pose
        """
        rospy.loginfo(">>> 收到检测请求")
        response = DetectSteelStampResponse()
        response.success = False
        
        # 1. 从缓存取数据
        rgb_msg = None
        depth_msg = None
        cam_info = None
        
        with self.data_lock:
            if self.latest_data["rgb"] is None:
                response.message = "No synchronized data received yet"
                rospy.logwarn(response.message)
                return response
            
            # 浅拷贝消息对象
            rgb_msg = self.latest_data["rgb"]
            depth_msg = self.latest_data["depth"]
            cam_info = self.latest_data["info"]
            
        # 2. 解析图像 (ROS msg -> Numpy)
        try:
            # 解析 RGB (BGR格式)
            rgb_arr = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(rgb_msg.height, rgb_msg.width, 3)
            # 解析 Depth (通常是 uint16, 单位 mm)
            depth_arr = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_msg.height, depth_msg.width)
        except Exception as e:
            response.message = f"Image parse error: {e}"
            return response

        # 3. 执行 PaddleOCR
        ocr_results = self.ocr.ocr(rgb_arr, cls=False)
        if not ocr_results or not ocr_results[0]:
            response.message = "OCR detected nothing"
            return response

        font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'

        if not os.path.exists(font_path):
            font_path = '/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf'

        if ocr_results and ocr_results[0]:
            boxes = [line[0] for line in ocr_results[0]]
            txts = [line[1][0] for line in ocr_results[0]]
            scores = [line[1][1] for line in ocr_results[0]]
            
            # 在图上画框
            im_show = draw_ocr(rgb_arr, boxes, txts, scores, font_path=font_path)
            # 转成 BGR 再保存
            im_show = cv2.cvtColor(im_show, cv2.COLOR_RGB2BGR)
            
            res_filename = f"result_1.jpg"
            cv2.imwrite(os.path.join(self.save_dir, res_filename), im_show)        


        # 4. 获取当前 TF 变换 (Base -> Camera)
        try:
            # rospy.Time(0) 获取最新的变换
            trans_stamped = self.tf_buffer.lookup_transform("base_link", rgb_msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            T_base_cam = self.ros_transform_to_matrix(trans_stamped.transform)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            response.message = f"TF Error: {e}"
            rospy.logerr(response.message)
            return response

        # 5. 遍历处理每个检测到的钢印
        detected_list = ocr_results[0]
        # 按 x 坐标排序
        detected_list.sort(key=lambda x: np.mean(np.array(x[0])[:, 0]))
        
        success_count = 0
        
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for line in detected_list:
            bbox = line[0] # [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
            text = line[1][0]
            
            # --- 核心调用：计算 3D 姿态 ---
            pose_msg = self.compute_stamp_pose(bbox, depth_arr, cam_info, T_base_cam)
            
            if pose_msg is not None:
                response.poses.append(pose_msg)
                response.texts.append(text)
                success_count += 1


                # ==========================================
                # [新增] 可视化 Marker 生成逻辑
                # ==========================================
                
                # 1. 箭头 Marker (显示姿态方向)
                arrow_marker = Marker()
                arrow_marker.header.frame_id = "base_link" # 确保和 pose_msg 的坐标系一致
                arrow_marker.header.stamp = rospy.Time.now()
                arrow_marker.ns = "steel_poses"
                arrow_marker.id = success_count * 2     # 保证 ID 不重复
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                arrow_marker.pose = pose_msg
                # 设置箭头尺寸 (长宽)
                arrow_marker.scale.x = 0.05 # 箭头长度 5cm
                arrow_marker.scale.y = 0.005
                arrow_marker.scale.z = 0.005
                # 设置颜色 (红色代表 X 轴方向，或者绿色代表法线，看你的 pose 定义)
                arrow_marker.color.r = 1.0
                arrow_marker.color.a = 1.0
                marker_array.markers.append(arrow_marker)
                
                # 2. 文字 Marker (显示识别到的数字)
                text_marker = Marker()
                text_marker.header.frame_id = "base_link"
                text_marker.header.stamp = rospy.Time.now()
                text_marker.ns = "steel_text"
                text_marker.id = success_count * 2 + 1  # ID 错开
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose = copy.deepcopy(pose_msg)
                text_marker.pose.position.z += 0.03 # 让文字悬浮在物体上方 3cm 处
                text_marker.text = f"[{text}]"
                text_marker.scale.z = 0.02 # 文字高度
                text_marker.color.b = 1.0  # 蓝色文字
                text_marker.color.g = 1.0
                text_marker.color.a = 1.0
                marker_array.markers.append(text_marker)
                
                # ==========================================
            else:
                rospy.logwarn(f"目标 '{text}' 3D计算失败 (可能是深度丢失或点云不足)")

        self.marker_pub.publish(marker_array)    

        if success_count > 0:
            response.success = True
            response.message = f"Success: {success_count} targets"
        else:
            response.message = "All targets failed in 3D processing"
        return response

    def compute_stamp_pose(self, bbox, depth_img, cam_info, T_base_cam):
        """
        输入: OCR包围盒, 深度图, 内参, 手眼矩阵
        输出: geometry_msgs/Pose (基座坐标系下的位姿)
        """
        # 1. 提取 ROI 并生成点云  astype：强制转换int
        box = np.array(bbox).astype(int)
        padding = 0 # 扩充一点边界以获取曲面信息
        xmin = max(0, np.min(box[:, 0]) - padding)
        xmax = min(depth_img.shape[1], np.max(box[:, 0]) + padding)
        ymin = max(0, np.min(box[:, 1]) - padding)
        ymax = min(depth_img.shape[0], np.max(box[:, 1]) + padding)
        
        roi_depth = depth_img[ymin:ymax, xmin:xmax]
        
        # 2. 内参解析 (P 矩阵通常是 3x4, P[0,0]=fx, P[1,1]=fy, P[0,2]=cx, P[1,2]=cy)
        fx, fy = cam_info.P[0], cam_info.P[5]
        cx, cy = cam_info.P[2], cam_info.P[6]

        # 3. 存储相机的内参、图像的宽高
        o3d_intrinsics = o3d.camera.PinholeCameraIntrinsic(
            width=depth_img.shape[1], height=depth_img.shape[0], 
            fx=fx, fy=fy, cx=cx, cy=cy
        )
        
        # 4. 生成局部点云 (ROI 区域)
        # 注意：这里需要先把 ROI 深度图放到全尺寸图的对应位置，或者手动计算偏移
    
        # 生成 ROI 区域的网格坐标
        u_grid, v_grid = np.meshgrid(np.arange(xmin, xmax), np.arange(ymin, ymax))
        # 有效深度范围 0.2m - 1.5m
        valid_mask = (roi_depth > 200) & (roi_depth < 1500) 
        
        if np.sum(valid_mask) < 20: # 有效点太少
            return None
            
        z_c = roi_depth[valid_mask].astype(np.float32) / 1000.0 # mm -> m
        u_valid = u_grid[valid_mask]
        v_valid = v_grid[valid_mask]
        
        # 相机系坐标下的 X,Y,Z
        x_c = (u_valid - cx) * z_c / fx
        y_c = (v_valid - cy) * z_c / fy
        points_cam = np.stack((x_c, y_c, z_c), axis=-1)
        
        # 5. Open3D 格式处理
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_cam)
        
        pcd.transform(T_base_cam)

        #使用RANSAC平面拟合

        if len(pcd.points) < 10: return None
        #distance_threshold=0.005: 容忍度 5mm,z距离平面5mm内的点才算;ransac_n=3: 每次随机抽 3 个点定平面;num_iterations=1000: 迭代 1000 次
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.005, ransac_n=3, num_iterations=1000)

        [a,b,c,d] = plane_model

        normal = np.array([a,b,c])
        normal = normal / np.linalg.norm(normal)

        # RANSAC 算出的法向量方向是随机的(可能朝上可能朝下) 我们需要根据实际物理场景纠正它。
        # 假设：钢印打在平面上，平面法向量应该大体指向 Z 轴正向(向上)
        if normal[2] < 0:
            normal = -normal
        
        #计算修正后的中心点解决深度估计问题

        pcd_inliers = pcd.select_by_index(inliers)
        center_raw = pcd_inliers.get_center()
        x_center, y_center = center_raw[0], center_raw[1]

        # [数学修正] 使用平面方程计算精确的 Z 值
        # z = -(ax + by + d) / c
        if abs(c) > 0.01: # 避免除以 0 (只有当平面完全垂直于地面时 c 才为 0)
            z_corrected = -(a * x_center + b * y_center + d) / c
        else:
            print("Warning: Plane normal too close to horizontal, using raw center z")
            z_corrected = center_raw[2] # 万一真的垂直了，退化回平均值
            
        center_pos = np.array([x_center, y_center, z_corrected])
 
        target_z = -normal 
        
        # 选取辅助轴 X，计算正交的 Y 和 X
        desired_x = np.array([1, 0, 0])
        # 防止死锁：如果法向量刚好也就是 X 轴，就换个辅助轴
        if abs(np.dot(target_z, desired_x)) > 0.95:
            desired_x = np.array([0, 1, 0])
            
        target_y = np.cross(target_z, desired_x)
        target_y = target_y / np.linalg.norm(target_y)
        
        target_x = np.cross(target_y, target_z) 
        target_x = target_x / np.linalg.norm(target_x)
        
        # 组合旋转矩阵
        R_matrix = np.column_stack((target_x, target_y, target_z))
        quat = R.from_matrix(R_matrix).as_quat()
        # 额外：将四元数转换为 RPY 欧拉角并打印（便于调试）
        try:
            rpy_deg = R.from_quat(quat).as_euler('xyz', degrees=False)
            rospy.loginfo(f"Quat -> RPY (deg): {rpy_deg}, Center Pos: {center_pos}")
        except Exception as e:
            rospy.logwarn(f"Failed to convert quat to RPY: {e}")

        # 8. 封装 ROS 消息
        pose = Pose()
        pose.position.x = center_pos[0]
        pose.position.y = center_pos[1]
        pose.position.z = center_pos[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        return pose

    def ros_transform_to_matrix(self, trans):
        """将 ROS Transform 消息转换为 4x4 矩阵"""
        q = trans.rotation
        t = trans.translation
        mat = np.eye(4)
        mat[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        mat[:3, 3] = [t.x, t.y, t.z]
        return mat

if __name__ == "__main__":
    try:
        server = SteelStamp3DServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass       

         # 6. 点云滤波与法向量
        # # 6.1 统计滤波去噪 (去除金属反光飞点)
        # pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        # if len(pcd.points) < 10:
        #     return None
        # # 6.2 估计法向量
        # pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
        
        # # 7. 计算最终 Pose
        # # 位置：取点云重心
        # points_base = np.asarray(pcd.points)  
        # center_pos = np.mean(points_base, axis=0) # 将所有点的坐标加起来除以总数，获得平均值,这种策略优于直接取OCR 框中心点
        # # 姿态：取平均法向量
        # normals_base = np.asarray(pcd.normals)  
        # avg_normal = np.mean(normals_base, axis=0)
        # avg_normal = avg_normal / np.linalg.norm(avg_normal) # 归一化