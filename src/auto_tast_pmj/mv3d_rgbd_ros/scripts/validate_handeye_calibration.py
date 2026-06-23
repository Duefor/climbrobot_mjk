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
    translation: np.ndarray
    quaternion: np.ndarray
    camera_translation: np.ndarray
    camera_quaternion: np.ndarray
    effector_translation: np.ndarray
    effector_quaternion: np.ndarray


class HandeyeCalibrationValidator:
    def __init__(self):
        rospy.init_node('handeye_calibration_validator', anonymous=False)

        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.board_frame = rospy.get_param('~board_frame', 'chessboard_board')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
        self.effector_frame = rospy.get_param('~effector_frame', 'Link_5')
        self.lookup_timeout = float(rospy.get_param('~lookup_timeout', 1.0))
        self.min_samples = int(rospy.get_param('~min_samples', 8))
        self.good_translation_std_mm = float(rospy.get_param('~good_translation_std_mm', 2.0))
        self.good_translation_max_mm = float(rospy.get_param('~good_translation_max_mm', 5.0))
        self.good_rotation_mean_deg = float(rospy.get_param('~good_rotation_mean_deg', 0.8))
        self.good_rotation_max_deg = float(rospy.get_param('~good_rotation_max_deg', 1.5))

        script_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_root = os.path.dirname(script_dir)
        self.output_dir = rospy.get_param('~output_dir', os.path.join(pkg_root, 'debug_images'))
        os.makedirs(self.output_dir, exist_ok=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.samples = []

    def lookup_transform(self, target_frame, source_frame):
        trans = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rospy.Time(0),
            rospy.Duration(self.lookup_timeout),
        )
        translation = np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z,
        ], dtype=np.float64)
        quaternion = np.array([
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w,
        ], dtype=np.float64)
        return translation, quaternion

    def take_sample(self):
        board_translation, board_quaternion = self.lookup_transform(self.base_frame, self.board_frame)
        camera_translation, camera_quaternion = self.lookup_transform(self.base_frame, self.camera_frame)
        effector_translation, effector_quaternion = self.lookup_transform(self.base_frame, self.effector_frame)

        sample = TransformSample(
            index=len(self.samples) + 1,
            timestamp=time.time(),
            translation=board_translation,
            quaternion=board_quaternion,
            camera_translation=camera_translation,
            camera_quaternion=camera_quaternion,
            effector_translation=effector_translation,
            effector_quaternion=effector_quaternion,
        )
        self.samples.append(sample)
        return sample

    def compute_report(self):
        translations = np.stack([sample.translation for sample in self.samples], axis=0)
        rotations = R.from_quat(np.stack([sample.quaternion for sample in self.samples], axis=0))

        mean_translation = np.mean(translations, axis=0)
        translation_errors = np.linalg.norm(translations - mean_translation, axis=1)
        translation_std_xyz = np.std(translations, axis=0)

        mean_rotation = rotations.mean()
        rel_rotations = mean_rotation.inv() * rotations
        rotation_errors_deg = np.degrees(np.abs(rel_rotations.magnitude()))

        report = {
            'sample_count': len(self.samples),
            'base_frame': self.base_frame,
            'board_frame': self.board_frame,
            'camera_frame': self.camera_frame,
            'effector_frame': self.effector_frame,
            'mean_translation_m': mean_translation.tolist(),
            'translation_std_xyz_mm': (translation_std_xyz * 1000.0).tolist(),
            'translation_error_mean_mm': float(np.mean(translation_errors) * 1000.0),
            'translation_error_max_mm': float(np.max(translation_errors) * 1000.0),
            'rotation_error_mean_deg': float(np.mean(rotation_errors_deg)),
            'rotation_error_max_deg': float(np.max(rotation_errors_deg)),
        }
        report['assessment'] = self.assess(report)
        return report

    def assess(self, report):
        if report['sample_count'] < self.min_samples:
            return '样本数不足，结论不可靠'

        max_std_mm = max(report['translation_std_xyz_mm'])
        if (
            max_std_mm <= self.good_translation_std_mm
            and report['translation_error_max_mm'] <= self.good_translation_max_mm
            and report['rotation_error_mean_deg'] <= self.good_rotation_mean_deg
            and report['rotation_error_max_deg'] <= self.good_rotation_max_deg
        ):
            return '标定结果较稳定，可用于高精度任务'

        if (
            max_std_mm <= self.good_translation_std_mm * 2.0
            and report['translation_error_max_mm'] <= self.good_translation_max_mm * 2.0
            and report['rotation_error_mean_deg'] <= self.good_rotation_mean_deg * 2.0
        ):
            return '标定结果可用，但精度一般，建议优化采样和检测条件'

        return '标定结果波动较大，建议重新检查手眼标定、板子固定、相机时间同步和检测质量'

    def save_report(self, report):
        stamp = time.strftime('%Y%m%d_%H%M%S')
        yaml_path = os.path.join(self.output_dir, 'handeye_validation_{}.yaml'.format(stamp))
        csv_path = os.path.join(self.output_dir, 'handeye_validation_{}.csv'.format(stamp))

        with open(yaml_path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(report, f, allow_unicode=True, sort_keys=False)

        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                'index', 'timestamp',
                'board_x', 'board_y', 'board_z', 'board_qx', 'board_qy', 'board_qz', 'board_qw',
                'camera_x', 'camera_y', 'camera_z', 'camera_qx', 'camera_qy', 'camera_qz', 'camera_qw',
                'effector_x', 'effector_y', 'effector_z', 'effector_qx', 'effector_qy', 'effector_qz', 'effector_qw',
            ])
            for sample in self.samples:
                writer.writerow([
                    sample.index,
                    sample.timestamp,
                    *sample.translation.tolist(),
                    *sample.quaternion.tolist(),
                    *sample.camera_translation.tolist(),
                    *sample.camera_quaternion.tolist(),
                    *sample.effector_translation.tolist(),
                    *sample.effector_quaternion.tolist(),
                ])

        return yaml_path, csv_path

    def print_intro(self):
        print('\n=== 手眼标定验证脚本 ===')
        print('用途：在 ChArUco 板固定不动的前提下，采样 base_link -> charuco_board 的稳定性。')
        print('如果标定准确，机械臂从不同姿态观察同一块板时，base_link -> charuco_board 应基本不变。')
        print('')
        print('启动前请确保：')
        print('1. 已启动 roscore')
        print('2. 已启动 demo.launch，并发布 Link_5 -> camera_color_optical_frame')
        print('3. 已启动 RealSense 相机节点')
        print('4. 已启动 charuco_publisher.py，且 tf 中可见 charuco_board')
        print('')
        print('操作方法：')
        print('- 每把机械臂移动到一个新的观测姿态，按一次回车采样')
        print('- 输入 q 后结束采样并输出统计结果')
        print('- 建议采集 8~15 个姿态，姿态变化要充分，板子保持固定')
        print('')

    def print_sample(self, sample):
        translation_mm = sample.translation * 1000.0
        print(
            '采样 #{:02d}: board in base = [{:.2f}, {:.2f}, {:.2f}] mm'.format(
                sample.index,
                translation_mm[0],
                translation_mm[1],
                translation_mm[2],
            )
        )

    def print_report(self, report, yaml_path, csv_path):
        print('\n=== 标定验证结果 ===')
        print('样本数: {}'.format(report['sample_count']))
        print('base -> board 平移均值 (m): {}'.format(np.round(report['mean_translation_m'], 6).tolist()))
        print('base -> board 各轴标准差 (mm): {}'.format(np.round(report['translation_std_xyz_mm'], 3).tolist()))
        print('平移误差均值 (mm): {:.3f}'.format(report['translation_error_mean_mm']))
        print('平移误差最大值 (mm): {:.3f}'.format(report['translation_error_max_mm']))
        print('旋转误差均值 (deg): {:.3f}'.format(report['rotation_error_mean_deg']))
        print('旋转误差最大值 (deg): {:.3f}'.format(report['rotation_error_max_deg']))
        print('结论: {}'.format(report['assessment']))
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
        validator = HandeyeCalibrationValidator()
        validator.run()
    except rospy.ROSInterruptException:
        pass
