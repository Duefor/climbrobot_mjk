#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import tf
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class ChessboardTF:
    def __init__(self):
        rospy.init_node("chessboard_tf")

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_cb)
        self.info_sub  = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.info_cb)

        self.br = tf.TransformBroadcaster()

        # ===== 参数 =====
        self.board_size = (11, 8)      # 内角点
        self.square_size = 0.03     # 米

        self.camera_matrix = None
        self.dist_coeffs = None

        # 棋盘格世界坐标
        objp = np.zeros((self.board_size[0]*self.board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.board_size[0], 0:self.board_size[1]].T.reshape(-1, 2)
        self.objp = objp * self.square_size

    def info_cb(self, msg):
        # 相机内参
        self.camera_matrix = np.array(msg.K).reshape(3,3)
        self.dist_coeffs = np.array(msg.D)

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, self.board_size, None)

        if not ret:
            cv2.imshow("view", frame)
            cv2.waitKey(1)
            return

        # 亚像素优化
        corners = cv2.cornerSubPix(
            gray, corners, (11,11), (-1,-1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )

        # 位姿求解
        ret, rvec, tvec = cv2.solvePnP(self.objp, corners,
                                       self.camera_matrix, self.dist_coeffs)

        if not ret:
            return

        # ===== TF发布 =====
        R, _ = cv2.Rodrigues(rvec)
        T = tvec.reshape(3)

        T_mat = np.eye(4)
        T_mat[:3,:3] = R

        quat = tf.transformations.quaternion_from_matrix(T_mat)

        self.br.sendTransform(
            T,
            quat,
            rospy.Time.now(),
            "chessboard",
            "camera_link"
        )

        # ===== 可视化 =====
        cv2.drawChessboardCorners(frame, self.board_size, corners, ret)

        # 坐标轴（长度=3个格子）
        axis = np.float32([
            [0,0,0],
            [3*self.square_size,0,0],
            [0,3*self.square_size,0],
            [0,0,-3*self.square_size]
        ])

        imgpts, _ = cv2.projectPoints(axis, rvec, tvec,
                                      self.camera_matrix, self.dist_coeffs)

        imgpts = imgpts.astype(int)

        origin = tuple(imgpts[0].ravel())
        frame = cv2.line(frame, origin, tuple(imgpts[1].ravel()), (0,0,255), 3) # X 红
        frame = cv2.line(frame, origin, tuple(imgpts[2].ravel()), (0,255,0), 3) # Y 绿
        frame = cv2.line(frame, origin, tuple(imgpts[3].ravel()), (255,0,0), 3) # Z 蓝

        cv2.imshow("view", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    print("Starting chessboard TF node...")
    ChessboardTF()
    rospy.spin()