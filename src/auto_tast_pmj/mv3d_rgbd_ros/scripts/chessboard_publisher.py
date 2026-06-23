#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys

ros_path = '/opt/ros/noetic/lib/python3/dist-packages'
sys_path = '/usr/lib/python3/dist-packages'

if os.path.exists(ros_path) and ros_path not in sys.path:
    sys.path.append(ros_path)

if os.path.exists(sys_path) and sys_path not in sys.path:
    sys.path.append(sys_path)

import cv2
import numpy as np
import rospy
import tf2_ros
import geometry_msgs.msg
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image


class ChessboardTFPublisher:
    def __init__(self):
        rospy.init_node('chessboard_tf_publisher')

        self.inner_corners_x = int(rospy.get_param('~inner_corners_x', 11))
        self.inner_corners_y = int(rospy.get_param('~inner_corners_y', 8))
        self.square_size = float(rospy.get_param('~square_size', 0.03))
        self.img_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        self.info_topic = rospy.get_param('~camera_info_topic', '/camera/rgb/camera_info')
        self.board_frame_id = rospy.get_param('~board_frame_id', 'chessboard_board')
        self.use_find_sb = bool(rospy.get_param('~use_find_chessboard_sb', True))
        self.show_debug_view = bool(rospy.get_param('~show_debug_view', True))
        self.publish_debug_image = bool(rospy.get_param('~publish_debug_image', False))
        self.debug_topic = rospy.get_param('~debug_topic', '/debug/chessboard_image')
        self.min_corner_count = int(rospy.get_param('~min_corner_count', 8))
        self.solvepnp_flag = int(rospy.get_param('~solvepnp_flag', int(cv2.SOLVEPNP_ITERATIVE)))

        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1) if self.publish_debug_image else None

        self.camera_matrix = None
        self.dist_coeffs = None

        self.pattern_size = (self.inner_corners_x, self.inner_corners_y)
        self.object_points = self.build_object_points()

        rospy.Subscriber(self.info_topic, CameraInfo, self.info_cb, queue_size=1)
        rospy.Subscriber(self.img_topic, Image, self.image_cb, queue_size=1)

        rospy.loginfo('Chessboard detector started')
        rospy.loginfo('pattern=%sx%s, square_size=%.6fm, board_frame=%s',
                      self.inner_corners_x, self.inner_corners_y, self.square_size, self.board_frame_id)
        rospy.loginfo('topics: image=%s, camera_info=%s', self.img_topic, self.info_topic)

    def build_object_points(self):
        object_points = np.zeros((self.inner_corners_x * self.inner_corners_y, 3), np.float32)
        grid = np.mgrid[0:self.inner_corners_x, 0:self.inner_corners_y].T.reshape(-1, 2)
        object_points[:, :2] = grid * self.square_size
        return object_points

    def info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D, dtype=np.float64)
            rospy.loginfo('Camera info received. Ready to detect chessboard.')

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            rospy.logerr_throttle(5.0, 'CV bridge conversion failed: %s', exc)
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = self.find_corners(gray)

        if not found or corners is None or len(corners) < self.min_corner_count:
            self.publish_debug(image, msg.header)
            self.show_image(image)
            return

        corners = self.refine_corners(gray, corners)

        success, rvec, tvec = cv2.solvePnP(
            self.object_points,
            corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=self.solvepnp_flag,
        )

        if not success:
            rospy.logwarn_throttle(5.0, 'solvePnP failed for chessboard')
            self.publish_debug(image, msg.header)
            self.show_image(image)
            return

        self.publish_tf(rvec, tvec, msg.header.frame_id, msg.header.stamp)
        self.draw_debug(image, corners, rvec, tvec)
        self.publish_debug(image, msg.header)
        self.show_image(image)

    def find_corners(self, gray):
        if self.use_find_sb and hasattr(cv2, 'findChessboardCornersSB'):
            flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
            found, corners = cv2.findChessboardCornersSB(gray, self.pattern_size, flags)
            if found:
                return found, corners

        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
        return cv2.findChessboardCorners(gray, self.pattern_size, flags)

    def refine_corners(self, gray, corners):
        if corners is None:
            return None
        if self.use_find_sb and hasattr(cv2, 'findChessboardCornersSB'):
            return corners
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
        return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    def publish_tf(self, rvec, tvec, parent_frame, timestamp):
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = self.board_frame_id
        transform.transform.translation.x = float(tvec[0][0])
        transform.transform.translation.y = float(tvec[1][0])
        transform.transform.translation.z = float(tvec[2][0])

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quat = R.from_matrix(rotation_matrix).as_quat()
        transform.transform.rotation.x = float(quat[0])
        transform.transform.rotation.y = float(quat[1])
        transform.transform.rotation.z = float(quat[2])
        transform.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(transform)

    def draw_debug(self, image, corners, rvec, tvec):
        cv2.drawChessboardCorners(image, self.pattern_size, corners, True)
        try:
            cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
        except Exception:
            pass

    def publish_debug(self, image, header):
        if self.debug_pub is None:
            return
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            debug_msg.header = header
            self.debug_pub.publish(debug_msg)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, 'Failed to publish debug chessboard image: %s', exc)

    def show_image(self, image):
        if not self.show_debug_view:
            return
        cv2.imshow('Chessboard Detection View', image)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        ChessboardTFPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
