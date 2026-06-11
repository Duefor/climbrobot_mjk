#!/usr/bin/env python3
"""
scan_environment_server 动作服务器测试节点。

发送一次扫描请求，等待结果，打印每个采样点的位姿。
"""

import sys
import rospy
import actionlib
from realsense_set.msg import ScanEnvironmentAction, ScanEnvironmentGoal


def main():
    rospy.init_node("scan_test_client")

    # 连接到动作服务器
    client = actionlib.SimpleActionClient("scan_environment", ScanEnvironmentAction)
    rospy.loginfo("等待 scan_environment 服务器...")
    if not client.wait_for_server(rospy.Duration(10.0)):
        rospy.logerr("超时：未连接到 scan_environment 服务器")
        sys.exit(1)
    rospy.loginfo("已连接到 scan_environment 服务器")

    # ---- 构造目标 ----
    goal = ScanEnvironmentGoal()
    # 非零值覆盖 ROS param，0 表示使用默认值
    goal.grid_rows = 0       # 0 = 使用 ~grid_rows 默认值
    goal.grid_cols = 0
    goal.window_size = 0
    goal.max_depth = 0.0

    # ---- 发送异步请求 ----
    rospy.loginfo("发送扫描请求...")
    client.send_goal(goal)

    # ---- 等待并打印反馈 ----
    rate = rospy.Rate(20)
    TERMINAL = {
        actionlib.GoalStatus.SUCCEEDED,
        actionlib.GoalStatus.ABORTED,
        actionlib.GoalStatus.PREEMPTED,
        actionlib.GoalStatus.REJECTED,
        actionlib.GoalStatus.RECALLED,
        actionlib.GoalStatus.LOST,
    }
    while client.get_state() not in TERMINAL:
        state = client.get_state()
        state_str = {
            actionlib.GoalStatus.PENDING: "PENDING",
            actionlib.GoalStatus.ACTIVE: "ACTIVE",
            actionlib.GoalStatus.PREEMPTED: "PREEMPTED",
            actionlib.GoalStatus.SUCCEEDED: "SUCCEEDED",
            actionlib.GoalStatus.ABORTED: "ABORTED",
        }.get(state, "UNKNOWN({})".format(state))
        rospy.loginfo_throttle(1.0, "状态: %s", state_str)
        rate.sleep()

    result = client.get_result()

    if result is None:
        rospy.logerr("扫描失败：无结果返回")
        sys.exit(1)

    if not result.samples:
        rospy.logwarn("扫描完成但没有采样点（可能深度图无效）")
        rospy.loginfo("消息: %s", result.message)
        sys.exit(0)

    rospy.loginfo("扫描完成: %s", result.message)
    rospy.loginfo("共 %d 个采样点:", len(result.samples))

    # ---- 打印前 5 个 + 统计 ----
    for i, pose in enumerate(result.samples[:5]):
        rospy.loginfo(
            "  [%d] pos=(%.3f, %.3f, %.3f)  quat=(%.3f, %.3f, %.3f, %.3f)",
            i,
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        )

    if len(result.samples) > 5:
        rospy.loginfo("  ... 还有 %d 个采样点 (不再逐个打印)", len(result.samples) - 5)

    # ---- 统计位姿范围 ----
    xs = [p.position.x for p in result.samples]
    ys = [p.position.y for p in result.samples]
    zs = [p.position.z for p in result.samples]
    rospy.loginfo(
        "位置范围: x=[%.3f, %.3f]  y=[%.3f, %.3f]  z=[%.3f, %.3f]",
        min(xs), max(xs), min(ys), max(ys), min(zs), max(zs),
    )


if __name__ == "__main__":
    main()
