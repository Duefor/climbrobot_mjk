#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_test");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("planning_group");

    geometry_msgs::Pose pose;

    pose.position.x = 0.4;
    pose.position.y = 0.2;
    pose.position.z = 0.3;

    double rx = 3.14;
    double ry = 0.0;
    double rz = 0.0;

    double angle = sqrt(rx*rx + ry*ry + rz*rz);

    tf2::Quaternion q;

    if(angle < 1e-6)
    {
        q.setRPY(0,0,0);
    }
    else
    {
        q.setRotation(tf2::Vector3(rx/angle, ry/angle, rz/angle), angle);
    }

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    move_group.setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success =
        (move_group.plan(plan) ==
        moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if(success)
        ROS_INFO("Plan success");

    // move_group.execute(plan);
}