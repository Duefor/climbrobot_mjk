#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from mv3d_rgbd_ros.srv import DetectSteelStamp, DetectSteelStampRequest


def tf_broadcast_loop(base_frame, camera_frame, x, y, z, roll, pitch, yaw, rate_hz):
    """
    后台线程：持续广播 base_link → camera 的静态 TF 变换。
    模拟相机在机器人前方的固定安装位置，供离线调试使用。
    """
    broadcaster = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.frame_id = base_frame
    t.child_frame_id = camera_frame

    # 位姿在循环外计算好（静态变换）
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        broadcaster.sendTransform(t)
        rate.sleep()


def main():
    rospy.init_node('test_ocr_client')

    # ---- TF 参数 ------------------------------------------------
    base_frame   = rospy.get_param('~base_frame',   'base_link')
    camera_frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
    tf_x     = rospy.get_param('~tf_x',     0.2)   # 相机在 base_link 下的 X (米)
    tf_y     = rospy.get_param('~tf_y',     0.0)   # 相机在 base_link 下的 Y
    tf_z     = rospy.get_param('~tf_z',     0.3)   # 相机在 base_link 下的 Z (高度)
    tf_roll  = rospy.get_param('~tf_roll',  0.0)   # 绕 X 旋转 (弧度)
    tf_pitch = rospy.get_param('~tf_pitch', 0.0)   # 绕 Y 旋转 (弧度)
    tf_yaw   = rospy.get_param('~tf_yaw',   0.0)   # 绕 Z 旋转 (弧度)
    tf_rate  = rospy.get_param('~tf_rate',  10)    # TF 发布频率 (Hz)

    # ---- 启动 TF 广播线程 ---------------------------------------
    tf_thread = threading.Thread(
        target=tf_broadcast_loop,
        args=(base_frame, camera_frame, tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw, tf_rate),
        daemon=True,
    )
    tf_thread.start()
    rospy.loginfo(
        'TF 广播已启动: %s → %s  pos=(%.2f, %.2f, %.2f) rpy=(%.2f, %.2f, %.2f) @ %d Hz',
        base_frame, camera_frame, tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw, tf_rate,
    )

    # ---- 服务调用 -----------------------------------------------
    service_name = rospy.get_param('~service_name', 'get_steel_stamp_location')

    rospy.loginfo('等待 OCR 服务: %s', service_name)
    try:
        rospy.wait_for_service(service_name, timeout=10.0)
    except rospy.ROSException:
        rospy.logerr('等待服务 %s 超时，请先启动 OCR 服务节点', service_name)
        return

    client = rospy.ServiceProxy(service_name, DetectSteelStamp)
    rospy.loginfo('服务 %s 已就绪', service_name)

    # 提示用户 RViz 可视化信息
    rospy.loginfo('=' * 60)
    rospy.loginfo('RViz 中查看可视化:')
    rospy.loginfo('  Fixed Frame:        %s', base_frame)
    rospy.loginfo('  Marker topic:       /debug/steel_markers_solo')
    rospy.loginfo('  PoseArray topic:    /debug/steel_pose_array_solo')
    rospy.loginfo('=' * 60)

    while not rospy.is_shutdown():
        # 等待用户按 Enter 后触发服务调用
        try:
            raw_input = raw_input  # Python 2 兼容
        except NameError:
            raw_input = input      # Python 3

        raw_input('\n按 Enter 键触发 OCR 检测 (Ctrl+C 退出)...')

        req = DetectSteelStampRequest()
        try:
            resp = client(req)
        except rospy.ServiceException as exc:
            rospy.logerr('服务调用失败: %s', exc)
            continue

        if resp.success:
            rospy.loginfo('✅ 检测成功!')
            rospy.loginfo('   识别文本: %s', ''.join(resp.texts))
            rospy.loginfo('   字符列表: %s', resp.texts)
            rospy.loginfo('   姿态数量: %d', len(resp.poses))
            for i, pose in enumerate(resp.poses):
                p = pose.position
                rospy.loginfo(
                    '   [%3d] pos=(%.4f, %.4f, %.4f)',
                    i, p.x, p.y, p.z,
                )
        else:
            rospy.logwarn('❌ 检测失败: %s', resp.message)


if __name__ == '__main__':
    main()
