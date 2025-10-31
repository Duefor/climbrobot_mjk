#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <iostream>

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
void cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 6) return;
    Vector3d pos(msg->data[0],msg->data[1],msg->data[2]);
    Vector3d rot(msg->data[3],msg->data[4],msg->data[5]);
    Vector3d conver_pos = mapPosition(pos);
    Vector3d conver_rot = mapRotation(rot);

    std_msgs::Float64MultiArray robot_pose;
    robot_pose.data.resize(6);
    for(int i = 0; i < 3; i++)
    {
        robot_pose.data[i] = conver_pos[i];
    }
    for(int i = 3; i < 6; i++)
    {
        robot_pose.data[i] = conver_rot[i-3];
    }
    pub.publish(robot_pose);
    std::cout << "转换后的机器人末端姿态为：" << "[" << robot_pose.data[0] << "," << robot_pose.data[1] << "," << robot_pose.data[2] << "," 
        << robot_pose.data[3] << "," << robot_pose.data[4] << "," << robot_pose.data[5] << "]" << std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"pose_converter");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("controller_pose",10,cb);
    pub = nh.advertise<std_msgs::Float64MultiArray>("robot_pose",10);
    ros::spin();
    return 0;
}
