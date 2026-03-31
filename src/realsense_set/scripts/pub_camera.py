#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
# import sys
# sys.path.append('/usr/lib/python2.7/dist-packages')


import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image, Imu
try:
    import pyrealsense2 as rs
except ImportError as exc:
    rs = None
    REALSENSE_IMPORT_ERROR = exc
else:
    REALSENSE_IMPORT_ERROR = None


class RealSenseD435iPublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.pipeline = None
        self.align = None
        self.depth_scale = 0.001
        self.color_info_template = None
        self.depth_info_template = None

        self.color_width = int(rospy.get_param('~color_width', 1280))
        self.color_height = int(rospy.get_param('~color_height', 720))
        self.depth_width = int(rospy.get_param('~depth_width', 640))
        self.depth_height = int(rospy.get_param('~depth_height', 480))
        self.fps = int(rospy.get_param('~fps', 15))
        self.frame_id = rospy.get_param('~frame_id', 'camera_color_optical_frame')
        self.timeout_ms = int(rospy.get_param('~timeout_ms', 1000))
        self.align_depth_to_color = bool(rospy.get_param('~align_depth_to_color', True))
        self.serial_number = str(rospy.get_param('~serial_number', '')).strip()

        self.rgb_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=5)
        self.depth_pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=5)
        self.rgb_info_pub = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size=1)
        self.depth_info_pub = rospy.Publisher('/camera/depth/camera_info', CameraInfo, queue_size=1)
        self.imu_pub = rospy.Publisher('/camera/imu', Imu, queue_size=5)

        self.active_stream_config = None

    def _make_camera_info(self, intrinsics, width, height):
        camera_info = CameraInfo()
        camera_info.width = int(width)
        camera_info.height = int(height)
        camera_info.distortion_model = 'plumb_bob'

        fx = float(intrinsics.fx)
        fy = float(intrinsics.fy)
        ppx = float(intrinsics.ppx)
        ppy = float(intrinsics.ppy)
        coeffs = list(intrinsics.coeffs)

        camera_info.K = [fx, 0.0, ppx,
                         0.0, fy, ppy,
                         0.0, 0.0, 1.0]
        camera_info.P = [fx, 0.0, ppx, 0.0,
                         0.0, fy, ppy, 0.0,
                         0.0, 0.0, 1.0, 0.0]
        camera_info.R = [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]
        camera_info.D = [float(value) for value in coeffs[:5]]
        return camera_info

    def _iter_stream_candidates(self):
        requested = (self.color_width, self.color_height, self.depth_width, self.depth_height, self.fps)
        fallback_candidates = [
            requested,
            (self.color_width, self.color_height, 640, 480, self.fps),
            (self.color_width, self.color_height, 848, 480, min(self.fps, 10)),
            (1280, 720, 848, 480, 15),
            (1280, 720, 640, 480, 15),
            (640, 480, 640, 480, 30),
        ]

        seen = set()
        for candidate in fallback_candidates:
            if candidate in seen:
                continue
            seen.add(candidate)
            yield candidate

    def start(self):
        if rs is None:
            raise RuntimeError('pyrealsense2 未安装: {}'.format(REALSENSE_IMPORT_ERROR))

        start_errors = []
        profile = None
        for color_width, color_height, depth_width, depth_height, fps in self._iter_stream_candidates():
            pipeline = rs.pipeline()
            config = rs.config()
            if self.serial_number:
                config.enable_device(self.serial_number)
            config.enable_stream(rs.stream.color, color_width, color_height, rs.format.bgr8, fps)
            config.enable_stream(rs.stream.depth, depth_width, depth_height, rs.format.z16, fps)
            config.enable_stream(rs.stream.accel)
            config.enable_stream(rs.stream.gyro)

            try:
                profile = pipeline.start(config)
                self.pipeline = pipeline
                self.active_stream_config = {
                    'color_width': color_width,
                    'color_height': color_height,
                    'depth_width': depth_width,
                    'depth_height': depth_height,
                    'fps': fps,
                }
                break
            except RuntimeError as exc:
                start_errors.append(
                    '{}x{} color / {}x{} depth @ {}fps -> {}'.format(
                        color_width, color_height, depth_width, depth_height, fps, exc
                    )
                )

        if self.pipeline is None or profile is None:
            raise RuntimeError('无法启动 D435i，尝试过的配置如下:\n{}'.format('\n'.join(start_errors)))

        device = profile.get_device()
        device_name = device.get_info(rs.camera_info.name)
        serial = device.get_info(rs.camera_info.serial_number)
        try:
            firmware = device.get_info(rs.camera_info.firmware_version)
        except RuntimeError:
            firmware = 'unknown'

        depth_sensor = device.first_depth_sensor()
        self.depth_scale = float(depth_sensor.get_depth_scale())

        for sensor in device.query_sensors():
            if sensor.supports(rs.option.enable_auto_exposure):
                sensor.set_option(rs.option.enable_auto_exposure, 1)
            if sensor.supports(rs.option.enable_auto_white_balance):
                sensor.set_option(rs.option.enable_auto_white_balance, 1)

        if self.align_depth_to_color:
            self.align = rs.align(rs.stream.color)

        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()

        color_intrinsics = color_stream.get_intrinsics()
        depth_intrinsics = depth_stream.get_intrinsics()

        self.color_info_template = self._make_camera_info(
            color_intrinsics,
            color_stream.width(),
            color_stream.height(),
        )

        if self.align_depth_to_color:
            self.depth_info_template = self._make_camera_info(
                color_intrinsics,
                color_stream.width(),
                color_stream.height(),
            )
        else:
            self.depth_info_template = self._make_camera_info(
                depth_intrinsics,
                depth_stream.width(),
                depth_stream.height(),
            )

        rospy.loginfo(
            'RealSense 已启动: %s SN=%s FW=%s, color=%sx%s, depth=%sx%s @ %sfps, align_depth_to_color=%s, depth_scale=%.6f',
            device_name,
            serial,
            firmware,
            self.active_stream_config['color_width'],
            self.active_stream_config['color_height'],
            self.active_stream_config['depth_width'],
            self.active_stream_config['depth_height'],
            self.active_stream_config['fps'],
            self.align_depth_to_color,
            self.depth_scale,
        )

        if self.active_stream_config['color_width'] != self.color_width or self.active_stream_config['color_height'] != self.color_height \
                or self.active_stream_config['depth_width'] != self.depth_width or self.active_stream_config['depth_height'] != self.depth_height \
                or self.active_stream_config['fps'] != self.fps:
            rospy.logwarn(
                '请求配置 %sx%s / %sx%s @ %sfps 不可用，已回退到 %sx%s / %sx%s @ %sfps',
                self.color_width,
                self.color_height,
                self.depth_width,
                self.depth_height,
                self.fps,
                self.active_stream_config['color_width'],
                self.active_stream_config['color_height'],
                self.active_stream_config['depth_width'],
                self.active_stream_config['depth_height'],
                self.active_stream_config['fps'],
            )

    def _publish_camera_info(self, template, stamp, publisher):
        camera_info = copy.deepcopy(template)
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = self.frame_id
        publisher.publish(camera_info)

    def spin(self):
        while not rospy.is_shutdown():
            try:
                frames = self.pipeline.wait_for_frames(self.timeout_ms)
            except RuntimeError as exc:
                rospy.logwarn_throttle(5.0, '等待 RealSense 帧超时/失败: %s', exc)
                continue

            if self.align is not None:
                frames = self.align.process(frames)

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            accel_frame = frames.first_or_default(rs.stream.accel)
            gyro_frame  = frames.first_or_default(rs.stream.gyro)
            if not depth_frame or not color_frame:
                rospy.logwarn_throttle(5.0, '收到不完整帧，跳过当前周期')
                continue

            stamp = rospy.Time.now()

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            if depth_image.dtype != np.uint16:
                depth_image = depth_image.astype(np.uint16)

            if abs(self.depth_scale - 0.001) > 1e-6:
                depth_image = np.clip(
                    np.rint(depth_image.astype(np.float32) * self.depth_scale * 1000.0),
                    0,
                    np.iinfo(np.uint16).max,
                ).astype(np.uint16)

            rgb_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            rgb_msg.header.stamp = stamp
            rgb_msg.header.frame_id = self.frame_id
            self.rgb_pub.publish(rgb_msg)
            self._publish_camera_info(self.color_info_template, stamp, self.rgb_info_pub)

            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = self.frame_id
            self.depth_pub.publish(depth_msg)
            self._publish_camera_info(self.depth_info_template, stamp, self.depth_info_pub)

            if accel_frame and gyro_frame:
                imu_msg = Imu()
                imu_msg.header.stamp = stamp
                imu_msg.header.frame_id = "camera_imu_optical_frame"

                # 读取数据
                accel_data = accel_frame.as_motion_frame().get_motion_data()
                gyro_data  = gyro_frame.as_motion_frame().get_motion_data()

                # 加速度 (m/s^2)
                imu_msg.linear_acceleration.x = accel_data.x
                imu_msg.linear_acceleration.y = accel_data.y
                imu_msg.linear_acceleration.z = accel_data.z

                # 角速度 (rad/s)
                imu_msg.angular_velocity.x = gyro_data.x
                imu_msg.angular_velocity.y = gyro_data.y
                imu_msg.angular_velocity.z = gyro_data.z

                # D435i 没有姿态
                imu_msg.orientation_covariance[0] = -1

                # 协方差（先给默认值，后续可标定）
                imu_msg.angular_velocity_covariance = [
                    1e-4, 0, 0,
                    0, 1e-4, 0,
                    0, 0, 1e-4
                ]

                imu_msg.linear_acceleration_covariance = [
                    1e-2, 0, 0,
                    0, 1e-2, 0,
                    0, 0, 1e-2
                ]

                self.imu_pub.publish(imu_msg)

    def stop(self):
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None


def main():
    rospy.init_node('realsense_d435i_image_publisher', anonymous=False)

    if rs is None:
        rospy.logerr('无法导入 pyrealsense2: %s', REALSENSE_IMPORT_ERROR)
        rospy.logerr('请先安装 librealsense/python 绑定，例如 apt 安装 realsense2 或 pip 安装 pyrealsense2。')
        return

    publisher = RealSenseD435iPublisher()

    try:
        publisher.start()
        publisher.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    except Exception as exc:
        rospy.logerr('RealSense 发布节点异常退出: %s', exc)
    finally:
        publisher.stop()


if __name__ == '__main__':
    main()
