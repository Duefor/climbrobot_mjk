#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_test");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 创建 MoveGroup（名称与SRDF中的planning group一致）
    moveit::planning_interface::MoveGroupInterface move_group("planning_group");

    // 设置目标位姿
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.4;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.3;

    target_pose.orientation.w = 1.0;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;

    move_group.setPoseTarget(target_pose);

    // 规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if(success)
    {
        move_group.execute(plan);
    }

    ros::shutdown();
    return 0;
}