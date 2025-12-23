#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <iostream>
#include <robot_set/TCPState.h>

using namespace Eigen;
ros::Publisher pub;

Vector3d mapPosition(const Vector3d& pos)
{
    Vector3d conver_pos;
    Matrix3d T;
    T << 3.4 , 0.0 , 0.0,
         0.0 , 3.8 , 0.0,
         0.0 , 0.0 , 4.0;
    Vector3d b(0.0 , 0.0 , 0.0);
    conver_pos = T * pos + b;
    return conver_pos;
}
Vector3d mapRotation(const Vector3d& rot)
{
    Vector3d conver_rot;
    Matrix3d T;
    T << 1.0 , 0.0 , 0.0,
         0.0 , 1.0 , 0.0,
         0.0 , 0.0 , 1.0;
    conver_rot = T * rot;
    return conver_rot;
}
Vector3d mapLinearVelocity(const Vector3d& vel)
{
    Matrix3d T;
    T << 3.4 , 0.0 , 0.0,
         0.0 , 3.8 , 0.0,
         0.0 , 0.0 , 4.0;
    return T * vel;
}
void cb(const robot_set::TCPState::ConstPtr& msg)
{
    if(msg->position.size() != 6) return;
    if(msg->velocity.size() != 6) return;
    Vector3d pos(msg->position[0],msg->position[1],msg->position[2]);
    Vector3d rot(msg->position[3],msg->position[4],msg->position[5]);
    Vector3d vel_lin(msg->velocity[0], msg->velocity[1], msg->velocity[2]);

    Vector3d conver_pos = mapPosition(pos);
    Vector3d conver_rot = mapRotation(rot);
    Vector3d conver_vel_lin = mapLinearVelocity(vel_lin);

    robot_set::TCPState robot_pose;
    robot_pose.position.resize(6);
    for(int i = 0; i < 3; i++)
    {
        robot_pose.position[i] = conver_pos[i];
    }
    for(int i = 3; i < 6; i++)
    {
        robot_pose.position[i] = conver_rot[i-3];
    }

    robot_pose.velocity.resize(6);
    for(int i = 0; i < 3; i++)
    {
        robot_pose.velocity[i] = conver_vel_lin[i];
    }
    for(int i = 3; i < 6; i++)
    {
        robot_pose.velocity[i] = 0.0;
    }

    robot_pose.header.stamp = ros::Time::now();
    pub.publish(robot_pose);
    std::cout << "转换后的机器臂期望tcp姿态为：" << "[" << robot_pose.position[0] << "," << robot_pose.position[1] << "," << robot_pose.position[2] << "," 
        << robot_pose.position[3] << "," << robot_pose.position[4] << "," << robot_pose.position[5] << "]" << std::endl;
    std::cout << "转换后的机械臂期望tcp速度为：" << "[" << robot_pose.velocity[0] << "," << robot_pose.velocity[1] << "," << robot_pose.velocity[2] << "," 
        << robot_pose.velocity[3] << "," << robot_pose.velocity[4] << "," << robot_pose.velocity[5] << "]" << std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"pose_converter");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/phantom/pose",10,cb);  // 手柄姿态和速度话题
    pub = nh.advertise<robot_set::TCPState>("/desired_robot_sub",10);   // 机械臂期望姿态和速度
    ros::spin();
    return 0;
}
