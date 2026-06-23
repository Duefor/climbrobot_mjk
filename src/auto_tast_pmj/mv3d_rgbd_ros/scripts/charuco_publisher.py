#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R

class CharucoTFPublisher:
    def __init__(self):
        rospy.init_node('charuco_tf_publisher')

        # ==================================================================
        # 🟢 【用户配置区域】 请根据你打印的A4纸真实情况修改这里！
        # ==================================================================
        
        # 1. 字典 ID: 必须和你生成图片时选的一致 (通常是 DICT_4X4_50 或 5X5_100)
        # 如果你用 Calib.io 生成，默认可能是 DICT_4X4_50
        self.dict_id = aruco.DICT_5X5_100
        
        # 2. 棋盘布局: X方向格子数(列) 和 Y方向格子数(行)
        # 注意：这里的格子数是指“黑色+白色”方块的总数
        self.squares_x = 5
        self.squares_y = 7
        
        # 3. 物理尺寸 (单位: 米) —— 必须用卡尺测量打印出来的纸！
        # square_len: 大方格(黑色或白色)的边长
        self.square_len = 0.030   # 30mm -> 0.030m
        # marker_len: 内部二维码黑色部分的边长
        self.marker_len = 0.022   # 22mm -> 0.022m
        
        # 4. 相机话题名称
        # 请用 rostopic list 确认你的相机发布 RGB 图和 Info 的话题名
        self.img_topic = "/camera/rgb/image_raw"
        self.info_topic = "/camera/rgb/camera_info"
        
        # 5. 发布的 TF 名字 (标定板的坐标系名)
        # 这个名字必须和你 launch 文件里的 tracking_marker_frame 一致
        self.board_frame_id = "charuco_board"
        
        # ==================================================================
        
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # 初始化 ArUco 字典
        try:
            self.aruco_dict = aruco.getPredefinedDictionary(self.dict_id)
        except AttributeError:
            # 兼容旧版 OpenCV 写法
            self.aruco_dict = aruco.Dictionary_get(self.dict_id)

        # 初始化 ChArUco 标定板对象
        # 参数顺序：列数, 行数, 格子米, Marker米, 字典
        try:
            self.board = aruco.CharucoBoard_create(
                self.squares_x, self.squares_y, 
                self.square_len, self.marker_len, 
                self.aruco_dict
            )
        except AttributeError:
            self.board = aruco.CharucoBoard(
                (self.squares_x, self.squares_y), 
                self.square_len, self.marker_len, 
                self.aruco_dict
            )

        self.camera_matrix = None
        self.dist_coeffs = None

        # 订阅相机信息和图像
        rospy.Subscriber(self.info_topic, CameraInfo, self.info_cb)
        rospy.Subscriber(self.img_topic, Image, self.img_cb)

        rospy.loginfo(f"ChArUco Detector Started for {self.squares_x}x{self.squares_y} board...")
        rospy.loginfo(f"Square: {self.square_len}m, Marker: {self.marker_len}m")

    def info_cb(self, msg):
        """ 获取相机内参 """
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            rospy.loginfo("Camera Info Received! Ready to detect.")

    def img_cb(self, msg):
        """ 图像处理主循环 """
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # 1. 检测 ArUco Markers (基石)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict)

        if ids is not None and len(ids) > 0:
            # 2. 插值检测 ChArUco 角点 (高精度来源)
            # 这步会利用已知布局，把角点位置优化到亚像素级
            retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                corners, ids, gray, self.board
            )

            # 至少检测到 4 个角点才能解算位姿
            if charuco_corners is not None and len(charuco_corners) > 4:
                # 3. 解算位姿 (PnP)
                valid, rvec, tvec = aruco.estimatePoseCharucoBoard(
                    charuco_corners, charuco_ids, self.board, 
                    self.camera_matrix, self.dist_coeffs, None, None
                )

                if valid:
                    # 4. 发布 TF
                    # 注意：rvec 是旋转向量，tvec 是平移向量
                    self.publish_tf(rvec, tvec, msg.header.frame_id, msg.header.stamp)
                    
                    # (可选) 在图上画坐标轴，确认方向是否正确
                    # 红=X, 绿=Y, 蓝=Z
                    try:
                        aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1) # 0.1m axis length
                    except AttributeError:
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

        # 显示图像窗口 (调试用，标定完可以注释掉)
        cv2.imshow("ChArUco Detection View", cv_image)
        cv2.waitKey(1)

    def publish_tf(self, rvec, tvec, parent_frame, timestamp):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = parent_frame  # 通常是 camera_color_optical_frame
        t.child_frame_id = self.board_frame_id

        # 填充平移
        t.transform.translation.x = tvec[0][0]
        t.transform.translation.y = tvec[1][0]
        t.transform.translation.z = tvec[2][0]

        # 填充旋转 (旋转向量 -> 矩阵 -> 四元数)
        # 使用 scipy 库处理，比手动算更稳
        rot_mat, _ = cv2.Rodrigues(rvec)
        r = R.from_matrix(rot_mat)
        quat = r.as_quat() # 返回 [x, y, z, w]

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    try:
        CharucoTFPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass