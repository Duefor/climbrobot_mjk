#!/usr/bin/env python3
import sys
sys.path.append('/usr/lib/python2.7/dist-packages')
import rospy
import pyrealsense2 as rs
from sensor_msgs.msg import Imu

class ImuNode:
    def __init__(self):
        self.pub = rospy.Publisher('/camera/imu', Imu, queue_size=50)

        self.pipeline = rs.pipeline()

        self.last_accel = None
        self.last_accel_time = None

        config = rs.config()
        config.enable_stream(rs.stream.gyro)
        config.enable_stream(rs.stream.accel)

        self.pipeline.start(config, self.callback)

    def callback(self, frame):
        if not frame.is_motion_frame():
            return

        m = frame.as_motion_frame()
        data = m.get_motion_data()
        st = m.get_profile().stream_type()

        t = frame.get_timestamp() / 1000.0

        # ===== accel 更新缓存 =====
        if st == rs.stream.accel:
            self.last_accel = data
            self.last_accel_time = t
            return

        # ===== gyro 触发发布 =====
        if st == rs.stream.gyro:

            if self.last_accel is None:
                return

            msg = Imu()
            msg.header.stamp = rospy.Time.from_sec(t)
            msg.header.frame_id = "camera_imu_optical_frame"

            # gyro（当前）
            msg.angular_velocity.x = data.x
            msg.angular_velocity.y = data.y
            msg.angular_velocity.z = data.z

            # accel（最近一次）
            msg.linear_acceleration.x = self.last_accel.x
            msg.linear_acceleration.y = self.last_accel.y
            msg.linear_acceleration.z = self.last_accel.z

            msg.orientation_covariance[0] = -1

            self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("imu_test_node")
    node = ImuNode()
    rospy.spin()