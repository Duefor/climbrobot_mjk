#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
import sys
import time
from dataclasses import dataclass

ros_path = '/opt/ros/noetic/lib/python3/dist-packages'
sys_path = '/usr/lib/python3/dist-packages'

if os.path.exists(ros_path) and ros_path not in sys.path:
    sys.path.append(ros_path)

if os.path.exists(sys_path) and sys_path not in sys.path:
    sys.path.append(sys_path)

import numpy as np
import rospy
import tf2_ros
import yaml
from scipy.spatial.transform import Rotation as R


@dataclass
class TransformSample:
    index: int
    timestamp: float
    board_translation: np.ndarray
    board_quaternion: np.ndarray
    effector_to_camera_translation: np.ndarray
    effector_to_camera_quaternion: np.ndarray
    camera_translation: np.ndarray
    camera_quaternion: np.ndarray
    effector_translation: np.ndarray
    effector_quaternion: np.ndarray


class Link6HandeyeValidator:
    def __init__(self):
        rospy.init_node('link6_handeye_validator', anonymous=False)

        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.board_frame = rospy.get_param('~board_frame', 'chessboard_board')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
        self.effector_frame = rospy.get_param('~effector_frame', 'Link_6')
        self.lookup_timeout = float(rospy.get_param('~lookup_timeout', 1.0))
        self.min_samples = int(rospy.get_param('~min_samples', 8))

        self.good_board_translation_std_mm = float(rospy.get_param('~good_board_translation_std_mm', 2.0))
        self.good_board_translation_max_mm = float(rospy.get_param('~good_board_translation_max_mm', 5.0))
        self.good_board_rotation_mean_deg = float(rospy.get_param('~good_board_rotation_mean_deg', 0.8))
        self.good_board_rotation_max_deg = float(rospy.get_param('~good_board_rotation_max_deg', 1.5))

        self.good_rigidity_translation_std_mm = float(rospy.get_param('~good_rigidity_translation_std_mm', 1.0))
        self.good_rigidity_translation_max_mm = float(rospy.get_param('~good_rigidity_translation_max_mm', 2.0))
        self.good_rigidity_rotation_mean_deg = float(rospy.get_param('~good_rigidity_rotation_mean_deg', 0.5))
        self.good_rigidity_rotation_max_deg = float(rospy.get_param('~good_rigidity_rotation_max_deg', 1.0))

        script_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_root = os.path.dirname(script_dir)
        self.output_dir = rospy.get_param('~output_dir', os.path.join(pkg_root, 'debug_images'))
        os.makedirs(self.output_dir, exist_ok=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.samples = []

    def lookup_transform(self, target_frame, source_frame):
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rospy.Time(0),
            rospy.Duration(self.lookup_timeout),
        )
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=np.float64)
        quaternion = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ], dtype=np.float64)
        return translation, quaternion

    def take_sample(self):
        board_translation, board_quaternion = self.lookup_transform(self.base_frame, self.board_frame)
        effector_to_camera_translation, effector_to_camera_quaternion = self.lookup_transform(self.effector_frame, self.camera_frame)
        camera_translation, camera_quaternion = self.lookup_transform(self.base_frame, self.camera_frame)
        effector_translation, effector_quaternion = self.lookup_transform(self.base_frame, self.effector_frame)

        sample = TransformSample(
            index=len(self.samples) + 1,
            timestamp=time.time(),
            board_translation=board_translation,
            board_quaternion=board_quaternion,
            effector_to_camera_translation=effector_to_camera_translation,
            effector_to_camera_quaternion=effector_to_camera_quaternion,
            camera_translation=camera_translation,
            camera_quaternion=camera_quaternion,
            effector_translation=effector_translation,
            effector_quaternion=effector_quaternion,
        )
        self.samples.append(sample)
        return sample

    @staticmethod
    def compute_pose_stats(translations, quaternions):
        rotations = R.from_quat(quaternions)
        mean_translation = np.mean(translations, axis=0)
        translation_errors = np.linalg.norm(translations - mean_translation, axis=1)
        translation_std_xyz = np.std(translations, axis=0)

        mean_rotation = rotations.mean()
        rel_rotations = mean_rotation.inv() * rotations
        rotation_errors_deg = np.degrees(np.abs(rel_rotations.magnitude()))

        return {
            'mean_translation_m': mean_translation.tolist(),
            'translation_std_xyz_mm': (translation_std_xyz * 1000.0).tolist(),
            'translation_error_mean_mm': float(np.mean(translation_errors) * 1000.0),
            'translation_error_max_mm': float(np.max(translation_errors) * 1000.0),
            'rotation_error_mean_deg': float(np.mean(rotation_errors_deg)),
            'rotation_error_max_deg': float(np.max(rotation_errors_deg)),
        }

    def assess_board_stability(self, stats):
        max_std_mm = max(stats['translation_std_xyz_mm'])
        if (
            max_std_mm <= self.good_board_translation_std_mm
            and stats['translation_error_max_mm'] <= self.good_board_translation_max_mm
            and stats['rotation_error_mean_deg'] <= self.good_board_rotation_mean_deg
            and stats['rotation_error_max_deg'] <= self.good_board_rotation_max_deg
        ):
            return 'base->board 稳定，手眼结果在该测试下表现良好'
        return 'base->board 波动较大，说明整条视觉定位链仍存在明显误差'

    def assess_rigidity(self, stats):
        max_std_mm = max(stats['translation_std_xyz_mm'])
        if (
            max_std_mm <= self.good_rigidity_translation_std_mm
            and stats['translation_error_max_mm'] <= self.good_rigidity_translation_max_mm
            and stats['rotation_error_mean_deg'] <= self.good_rigidity_rotation_mean_deg
            and stats['rotation_error_max_deg'] <= self.good_rigidity_rotation_max_deg
        ):
            return 'effector->camera 基本刚性不变，可作为静态外参使用'
        return 'effector->camera 变化明显，不满足静态外参假设；若这是 Link_6，则通常意味着相机并不真正刚连在 Link_6 上'

    def compute_report(self):
        board_translations = np.stack([sample.board_translation for sample in self.samples], axis=0)
        board_quaternions = np.stack([sample.board_quaternion for sample in self.samples], axis=0)
        rigidity_translations = np.stack([sample.effector_to_camera_translation for sample in self.samples], axis=0)
        rigidity_quaternions = np.stack([sample.effector_to_camera_quaternion for sample in self.samples], axis=0)

        board_stats = self.compute_pose_stats(board_translations, board_quaternions)
        rigidity_stats = self.compute_pose_stats(rigidity_translations, rigidity_quaternions)

        report = {
            'sample_count': len(self.samples),
            'base_frame': self.base_frame,
            'board_frame': self.board_frame,
            'camera_frame': self.camera_frame,
            'effector_frame': self.effector_frame,
            'board_stability': board_stats,
            'effector_to_camera_rigidity': rigidity_stats,
            'board_assessment': self.assess_board_stability(board_stats),
            'rigidity_assessment': self.assess_rigidity(rigidity_stats),
        }

        if len(self.samples) < self.min_samples:
            report['overall_assessment'] = '样本数不足，结论不可靠'
        elif '不满足静态外参假设' in report['rigidity_assessment']:
            report['overall_assessment'] = 'Link_6->camera 不是稳定刚体变换，不建议直接把 Link_6 标定结果当静态手眼外参使用'
        elif '波动较大' in report['board_assessment']:
            report['overall_assessment'] = '外参刚性尚可，但 base->board 波动偏大，说明视觉检测或标定精度仍需提升'
        else:
            report['overall_assessment'] = 'Link_6 标定在本次测试下可用'

        return report

    def save_report(self, report):
        stamp = time.strftime('%Y%m%d_%H%M%S')
        yaml_path = os.path.join(self.output_dir, 'link6_handeye_validation_{}.yaml'.format(stamp))
        csv_path = os.path.join(self.output_dir, 'link6_handeye_validation_{}.csv'.format(stamp))

        with open(yaml_path, 'w', encoding='utf-8') as file:
            yaml.safe_dump(report, file, allow_unicode=True, sort_keys=False)

        with open(csv_path, 'w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow([
                'index', 'timestamp',
                'board_x', 'board_y', 'board_z', 'board_qx', 'board_qy', 'board_qz', 'board_qw',
                'e2c_x', 'e2c_y', 'e2c_z', 'e2c_qx', 'e2c_qy', 'e2c_qz', 'e2c_qw',
                'camera_x', 'camera_y', 'camera_z', 'camera_qx', 'camera_qy', 'camera_qz', 'camera_qw',
                'effector_x', 'effector_y', 'effector_z', 'effector_qx', 'effector_qy', 'effector_qz', 'effector_qw',
            ])
            for sample in self.samples:
                writer.writerow([
                    sample.index,
                    sample.timestamp,
                    *sample.board_translation.tolist(),
                    *sample.board_quaternion.tolist(),
                    *sample.effector_to_camera_translation.tolist(),
                    *sample.effector_to_camera_quaternion.tolist(),
                    *sample.camera_translation.tolist(),
                    *sample.camera_quaternion.tolist(),
                    *sample.effector_translation.tolist(),
                    *sample.effector_quaternion.tolist(),
                ])
        return yaml_path, csv_path

    def print_intro(self):
        print('\n=== Link_6 手眼标定验证脚本 ===')
        print('用途：在棋盘格固定不动的前提下，同时检查：')
        print('1. base_link -> chessboard_board 是否稳定')
        print('2. Link_6 -> camera_color_optical_frame 是否真的是固定刚体变换')
        print('')
        print('这个脚本特别适合验证一种高风险情况：')
        print('- 你把手眼标定 effector 设成 Link_6')
        print('- 但相机实际上不随 joint_6 的转动一起旋转')
        print('')
        print('启动前请确保：')
        print('1. 已启动 roscore')
        print('2. 已加载最新手眼外参')
        print('3. 已启动相机节点')
        print('4. 已启动 chessboard_publisher.py，且 TF 中可见 chessboard_board')
        print('')
        print('操作建议：')
        print('- 采样 8~15 次')
        print('- 尽量包含不同的 joint_6 姿态')
        print('- 棋盘格板保持完全不动')
        print('')

    def print_sample(self, sample):
        board_mm = sample.board_translation * 1000.0
        e2c_mm = sample.effector_to_camera_translation * 1000.0
        print(
            '采样 #{:02d}: board=[{:.2f}, {:.2f}, {:.2f}] mm | e2c=[{:.2f}, {:.2f}, {:.2f}] mm'.format(
                sample.index,
                board_mm[0], board_mm[1], board_mm[2],
                e2c_mm[0], e2c_mm[1], e2c_mm[2],
            )
        )

    def print_report(self, report, yaml_path, csv_path):
        board = report['board_stability']
        rigidity = report['effector_to_camera_rigidity']

        print('\n=== Link_6 标定验证结果 ===')
        print('样本数: {}'.format(report['sample_count']))
        print('')
        print('[1] base -> board 稳定性')
        print('  平移标准差 (mm): {}'.format(np.round(board['translation_std_xyz_mm'], 3).tolist()))
        print('  平移误差均值 (mm): {:.3f}'.format(board['translation_error_mean_mm']))
        print('  平移误差最大值 (mm): {:.3f}'.format(board['translation_error_max_mm']))
        print('  旋转误差均值 (deg): {:.3f}'.format(board['rotation_error_mean_deg']))
        print('  旋转误差最大值 (deg): {:.3f}'.format(board['rotation_error_max_deg']))
        print('  结论: {}'.format(report['board_assessment']))
        print('')
        print('[2] effector -> camera 刚性一致性')
        print('  平移标准差 (mm): {}'.format(np.round(rigidity['translation_std_xyz_mm'], 3).tolist()))
        print('  平移误差均值 (mm): {:.3f}'.format(rigidity['translation_error_mean_mm']))
        print('  平移误差最大值 (mm): {:.3f}'.format(rigidity['translation_error_max_mm']))
        print('  旋转误差均值 (deg): {:.3f}'.format(rigidity['rotation_error_mean_deg']))
        print('  旋转误差最大值 (deg): {:.3f}'.format(rigidity['rotation_error_max_deg']))
        print('  结论: {}'.format(report['rigidity_assessment']))
        print('')
        print('总体结论: {}'.format(report['overall_assessment']))
        print('YAML 报告: {}'.format(yaml_path))
        print('CSV 样本: {}'.format(csv_path))

    def run(self):
        self.print_intro()
        while not rospy.is_shutdown():
            user_input = input('移动机械臂到新姿态后按回车采样，输入 q 结束: ').strip().lower()
            if user_input == 'q':
                break
            try:
                sample = self.take_sample()
                self.print_sample(sample)
            except Exception as exc:
                print('采样失败: {}'.format(exc))

        if len(self.samples) == 0:
            print('未采集到任何样本，退出。')
            return

        report = self.compute_report()
        yaml_path, csv_path = self.save_report(report)
        self.print_report(report, yaml_path, csv_path)


if __name__ == '__main__':
    try:
        validator = Link6HandeyeValidator()
        validator.run()
    except rospy.ROSInterruptException:
        pass
