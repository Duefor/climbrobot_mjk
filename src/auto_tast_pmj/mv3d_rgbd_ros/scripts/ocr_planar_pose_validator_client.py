#!/home/barry/workspace/ws_moveit/venv_ocr/bin/python3
# -*- coding: utf-8 -*-

import math
import os
import sys
from typing import Optional, Tuple

ros_path = '/opt/ros/noetic/lib/python3/dist-packages'
sys_path = '/usr/lib/python3/dist-packages'

if os.path.exists(ros_path) and ros_path not in sys.path:
    sys.path.append(ros_path)

if os.path.exists(sys_path) and sys_path not in sys.path:
    sys.path.append(sys_path)

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R

from mv3d_rgbd_ros.srv import DetectSteelStamp, DetectSteelStampRequest


class OcrPlanarPoseValidatorClient:
    def __init__(self):
        rospy.init_node('ocr_planar_pose_validator_client', anonymous=False)

        self.service_name = rospy.get_param('~service_name', 'get_steel_stamp_location')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.world_frame = rospy.get_param('~world_frame', 'base_link')
        self.tcp_frame = rospy.get_param('~tcp_frame', 'ee_link')
        self.link5_frame = rospy.get_param('~link5_frame', 'Link_5')
        self.link6_frame = rospy.get_param('~link6_frame', 'Link_6')

        self.joint_state_topic = rospy.get_param('~joint_state_topic', '/joint_states')
        self.tcp_pose_topic = rospy.get_param('~tcp_pose_topic', '/tcp_pose')
        self.base_pose_topic = rospy.get_param('~base_pose_topic', '')

        self.expected_plane_height_in_base = float(rospy.get_param('~expected_plane_height_in_base', 0.0))
        self.plane_height_tolerance = float(rospy.get_param('~plane_height_tolerance', 0.01))
        self.plane_spread_tolerance = float(rospy.get_param('~plane_spread_tolerance', 0.008))
        self.expected_pose_z_axis_in_base = np.asarray(
            rospy.get_param('~expected_pose_z_axis_in_base', [0.0, 0.0, -1.0]),
            dtype=np.float64,
        )
        self.normal_alignment_min_abs = float(rospy.get_param('~normal_alignment_min_abs', 0.95))
        self.tcp_orientation_tolerance_deg = float(rospy.get_param('~tcp_orientation_tolerance_deg', 10.0))
        self.call_timeout = float(rospy.get_param('~call_timeout', 5.0))
        self.debug_copy_prefix = rospy.get_param('~debug_copy_prefix', 'COPY_DEBUG')
        self.tcp_pose_format = rospy.get_param('~tcp_pose_format', 'auto')

        self.latest_tcp_pose_msg = None
        self.latest_base_pose_msg = None
        self.latest_joint_state_msg = None

        if np.linalg.norm(self.expected_pose_z_axis_in_base) < 1e-9:
            self.expected_pose_z_axis_in_base = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        else:
            self.expected_pose_z_axis_in_base /= np.linalg.norm(self.expected_pose_z_axis_in_base)

        if self.tcp_pose_topic:
            rospy.Subscriber(self.tcp_pose_topic, Pose, self.tcp_pose_callback, queue_size=1)
            rospy.loginfo('订阅 TCP 位姿话题: %s', self.tcp_pose_topic)

        if self.joint_state_topic:
            rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_callback, queue_size=1)
            rospy.loginfo('订阅关节状态话题: %s', self.joint_state_topic)

        if self.base_pose_topic:
            rospy.Subscriber(self.base_pose_topic, PoseStamped, self.base_pose_callback, queue_size=1)
            rospy.loginfo('订阅 base 位姿话题: %s', self.base_pose_topic)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo('等待 OCR 服务: %s', self.service_name)
        rospy.wait_for_service(self.service_name, timeout=self.call_timeout)
        self.client = rospy.ServiceProxy(self.service_name, DetectSteelStamp)
        rospy.loginfo('OCR 平面位姿校验客户端已启动')

    def tcp_pose_callback(self, msg):
        self.latest_tcp_pose_msg = msg

    def joint_state_callback(self, msg):
        self.latest_joint_state_msg = msg

    def base_pose_callback(self, msg):
        self.latest_base_pose_msg = msg

    def lookup_pose_from_tf(self, target_frame: str, source_frame: str) -> Optional[PoseStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as exc:
            rospy.logwarn('TF 查询失败 %s <- %s: %s', target_frame, source_frame, exc)
            return None

        pose_stamped = PoseStamped()
        pose_stamped.header = transform.header
        pose_stamped.pose.position.x = transform.transform.translation.x
        pose_stamped.pose.position.y = transform.transform.translation.y
        pose_stamped.pose.position.z = transform.transform.translation.z
        pose_stamped.pose.orientation = transform.transform.rotation
        return pose_stamped

    def get_tcp_pose(self) -> Optional[PoseStamped]:
        if self.latest_tcp_pose_msg is not None:
            return self.convert_tcp_pose_to_pose_stamped(self.latest_tcp_pose_msg)
        return self.lookup_pose_from_tf(self.base_frame, self.tcp_frame)

    def get_base_pose(self) -> Optional[PoseStamped]:
        if self.latest_base_pose_msg is not None:
            return self.latest_base_pose_msg
        if self.world_frame == self.base_frame:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.base_frame
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.orientation.w = 1.0
            return pose_stamped
        return self.lookup_pose_from_tf(self.world_frame, self.base_frame)

    def call_ocr_service(self):
        request = DetectSteelStampRequest()
        return self.client(request)

    def pose_to_rotation(self, pose: Pose) -> R:
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        quat_norm = np.linalg.norm(quat)
        if quat_norm < 1e-9:
            return R.identity()
        quat = np.asarray(quat, dtype=np.float64) / quat_norm
        return R.from_quat(quat)

    def pose_stamped_to_rotation(self, pose_stamped: PoseStamped) -> R:
        pose = pose_stamped.pose
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        return R.from_quat(quat)

    def get_pose_z_axis(self, pose: Pose) -> np.ndarray:
        rotation = self.pose_to_rotation(pose)
        return rotation.as_matrix()[:, 2]

    def normalize_quaternion(self, quat: np.ndarray) -> np.ndarray:
        quat_norm = np.linalg.norm(quat)
        if quat_norm < 1e-9:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        return quat / quat_norm

    def infer_tcp_pose_format(self, tcp_pose_msg: Pose) -> str:
        if self.tcp_pose_format in ('quaternion', 'rotvec'):
            return self.tcp_pose_format

        quat = np.array([
            tcp_pose_msg.orientation.x,
            tcp_pose_msg.orientation.y,
            tcp_pose_msg.orientation.z,
            tcp_pose_msg.orientation.w,
        ], dtype=np.float64)
        quat_norm = np.linalg.norm(quat)

        if abs(quat_norm - 1.0) < 1e-3 and abs(tcp_pose_msg.orientation.w) <= 1.0:
            return 'quaternion'
        return 'rotvec'

    def convert_tcp_pose_to_pose_stamped(self, tcp_pose_msg: Pose) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position = tcp_pose_msg.position

        pose_format = self.infer_tcp_pose_format(tcp_pose_msg)
        if pose_format == 'quaternion':
            quat = self.normalize_quaternion(np.array([
                tcp_pose_msg.orientation.x,
                tcp_pose_msg.orientation.y,
                tcp_pose_msg.orientation.z,
                tcp_pose_msg.orientation.w,
            ], dtype=np.float64))
        else:
            rotvec = np.array([
                tcp_pose_msg.orientation.x,
                tcp_pose_msg.orientation.y,
                tcp_pose_msg.orientation.z,
            ], dtype=np.float64)
            rotvec_norm = np.linalg.norm(rotvec)
            if rotvec_norm < 1e-9:
                quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
            else:
                quat = R.from_rotvec(rotvec).as_quat()

        pose_stamped.pose.orientation.x = float(quat[0])
        pose_stamped.pose.orientation.y = float(quat[1])
        pose_stamped.pose.orientation.z = float(quat[2])
        pose_stamped.pose.orientation.w = float(quat[3])
        return pose_stamped

    def format_pose_copy_line(self, label: str, pose_stamped: Optional[PoseStamped]) -> str:
        if pose_stamped is None:
            return '{} {} unavailable'.format(self.debug_copy_prefix, label)

        pose = pose_stamped.pose
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        euler = self.pose_to_rotation(pose).as_euler('xyz', degrees=True)
        return (
            '{} {} frame={} pos=({:.6f},{:.6f},{:.6f}) '
            'quat=({:.6f},{:.6f},{:.6f},{:.6f}) rpy_deg=({:.3f},{:.3f},{:.3f})'
        ).format(
            self.debug_copy_prefix,
            label,
            pose_stamped.header.frame_id,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            quat[0],
            quat[1],
            quat[2],
            quat[3],
            euler[0],
            euler[1],
            euler[2],
        )

    def format_joint_state_copy_line(self) -> str:
        if self.latest_joint_state_msg is None:
            return '{} JOINT_STATES unavailable'.format(self.debug_copy_prefix)

        parts = []
        for name, position in zip(self.latest_joint_state_msg.name, self.latest_joint_state_msg.position):
            parts.append('{}={:.6f}'.format(name, position))
        return '{} JOINT_STATES {}'.format(self.debug_copy_prefix, ' '.join(parts))

    def print_failure_diagnostics(self, response, tcp_pose_stamped, base_pose_stamped):
        rospy.logwarn('=== 详细调试信息（便于复制） ===')
        rospy.logwarn(self.format_joint_state_copy_line())
        rospy.logwarn(self.format_pose_copy_line('BASE_POSE', base_pose_stamped))
        rospy.logwarn(self.format_pose_copy_line('TCP_POSE_FROM_TOPIC', tcp_pose_stamped))

        if self.latest_tcp_pose_msg is not None:
            raw = self.latest_tcp_pose_msg
            rospy.logwarn(
                '%s TCP_RAW format=%s pos=(%.6f,%.6f,%.6f) ori=(%.6f,%.6f,%.6f,%.6f)',
                self.debug_copy_prefix,
                self.infer_tcp_pose_format(raw),
                raw.position.x,
                raw.position.y,
                raw.position.z,
                raw.orientation.x,
                raw.orientation.y,
                raw.orientation.z,
                raw.orientation.w,
            )

        for frame_name in [self.link5_frame, self.link6_frame, self.tcp_frame, 'ee_link']:
            pose_stamped = self.lookup_pose_from_tf(self.base_frame, frame_name)
            rospy.logwarn(self.format_pose_copy_line(frame_name, pose_stamped))

        for index, pose in enumerate(response.poses):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.base_frame
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose = pose
            rospy.logwarn(self.format_pose_copy_line('OCR_POSE_{}'.format(index), pose_stamped))

    def compute_orientation_difference_deg(self, pose_a: Pose, pose_b: Pose) -> float:
        rot_a = self.pose_to_rotation(pose_a)
        rot_b = self.pose_to_rotation(pose_b)
        relative_rot = rot_a.inv() * rot_b
        return math.degrees(relative_rot.magnitude())

    def evaluate_poses(self, ocr_poses, tcp_pose_stamped: Optional[PoseStamped], base_pose_stamped: Optional[PoseStamped]):
        result = {
            'success': True,
            'messages': [],
        }

        if not ocr_poses:
            result['success'] = False
            result['messages'].append('OCR 未返回任何 pose')
            return result

        z_values = np.array([pose.position.z for pose in ocr_poses], dtype=np.float64)
        z_min = float(np.min(z_values))
        z_max = float(np.max(z_values))
        z_mean = float(np.mean(z_values))
        z_spread = z_max - z_min

        result['messages'].append(
            'OCR 点位 base 系高度: min={:.4f} max={:.4f} mean={:.4f} spread={:.4f} m'.format(
                z_min, z_max, z_mean, z_spread)
        )

        if abs(z_mean - self.expected_plane_height_in_base) > self.plane_height_tolerance:
            result['success'] = False
            result['messages'].append(
                '平均高度偏离期望平面过大: |{:.4f} - {:.4f}| > {:.4f} m'.format(
                    z_mean,
                    self.expected_plane_height_in_base,
                    self.plane_height_tolerance,
                )
            )
        else:
            result['messages'].append('平均高度满足平面假设')

        if z_spread > self.plane_spread_tolerance:
            result['success'] = False
            result['messages'].append(
                '各字符高度离散过大，疑似不共面: {:.4f} > {:.4f} m'.format(
                    z_spread,
                    self.plane_spread_tolerance,
                )
            )
        else:
            result['messages'].append('各字符高度离散较小，可视为共面')

        for index, pose in enumerate(ocr_poses):
            target_z = self.get_pose_z_axis(pose)
            alignment = float(np.dot(target_z, self.expected_pose_z_axis_in_base))
            if alignment < self.normal_alignment_min_abs:
                result['success'] = False
                result['messages'].append(
                    'Pose[{}] 姿态 z 轴与期望轴 {} 对齐不足: {:.4f} < {:.4f}'.format(
                        index,
                        self.expected_pose_z_axis_in_base.tolist(),
                        alignment,
                        self.normal_alignment_min_abs,
                    )
                )
            else:
                result['messages'].append(
                    'Pose[{}] 姿态 z 轴与期望轴 {} 对齐良好: {:.4f}'.format(
                        index,
                        self.expected_pose_z_axis_in_base.tolist(),
                        alignment,
                    )
                )

        if tcp_pose_stamped is not None:
            tcp_pose = tcp_pose_stamped.pose
            for index, pose in enumerate(ocr_poses):
                tcp_diff_deg = self.compute_orientation_difference_deg(pose, tcp_pose)
                result['messages'].append(
                    'Pose[{}] 与当前 TCP 姿态差: {:.2f} deg'.format(index, tcp_diff_deg)
                )
                if tcp_diff_deg > self.tcp_orientation_tolerance_deg:
                    result['success'] = False
                    result['messages'].append(
                        'Pose[{}] 与当前 TCP 姿态差过大: {:.2f} > {:.2f} deg'.format(
                            index,
                            tcp_diff_deg,
                            self.tcp_orientation_tolerance_deg,
                        )
                    )
        else:
            result['messages'].append('未获取到 TCP pose，跳过与当前 TCP 姿态的比较')

        if base_pose_stamped is not None:
            base_pos = base_pose_stamped.pose.position
            result['messages'].append(
                'base 位姿参考: frame={} pos=({:.4f}, {:.4f}, {:.4f})'.format(
                    base_pose_stamped.header.frame_id,
                    base_pos.x,
                    base_pos.y,
                    base_pos.z,
                )
            )
        else:
            result['messages'].append('未获取到 base pose，跳过世界系参考输出')

        return result

    def run_once(self):
        tcp_pose_stamped = self.get_tcp_pose()
        base_pose_stamped = self.get_base_pose()

        if tcp_pose_stamped is not None:
            tcp = tcp_pose_stamped.pose.position
            rospy.loginfo(
                '当前 TCP 位姿参考: frame=%s pos=(%.4f, %.4f, %.4f)',
                tcp_pose_stamped.header.frame_id,
                tcp.x,
                tcp.y,
                tcp.z,
            )
        else:
            rospy.logwarn('当前未获取到 TCP pose')

        response = self.call_ocr_service()
        if not response.success:
            rospy.logerr('OCR 服务失败: %s', response.message)
            return False

        rospy.loginfo('OCR 返回 %d 个目标: %s', len(response.poses), list(response.texts))
        evaluation = self.evaluate_poses(response.poses, tcp_pose_stamped, base_pose_stamped)

        if evaluation['success']:
            rospy.loginfo('=== OCR 平面位姿校验通过 ===')
        else:
            rospy.logerr('=== OCR 平面位姿校验失败 ===')

        for message in evaluation['messages']:
            rospy.loginfo(message)

        for index, pose in enumerate(response.poses):
            rospy.loginfo(
                'Pose[%d] pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)',
                index,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )

        if not evaluation['success']:
            self.print_failure_diagnostics(response, tcp_pose_stamped, base_pose_stamped)

        return evaluation['success']


if __name__ == '__main__':
    try:
        node = OcrPlanarPoseValidatorClient()
        node.run_once()
    except rospy.ROSInterruptException:
        pass
