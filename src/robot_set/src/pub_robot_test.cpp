#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <cstdlib>
#include <ctime>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <robot_set/TCPState.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_robot_test");
    ros::NodeHandle nh;

    ros::Publisher pub_joint = nh.advertise<sensor_msgs::JointState>("/joint_pos", 10);
    ros::Publisher pub_tcp = nh.advertise<robot_set::TCPState>("/phantom/pose", 10);

    ros::Rate rate(100);

    sensor_msgs::JointState joint_msg;
    joint_msg.position.resize(6);
    joint_msg.velocity.resize(6);
    joint_msg.name = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};   // 需要与urdf中关节名称一致

    robot_set::TCPState tcp_msg;
    tcp_msg.position.resize(6);
    tcp_msg.velocity.resize(6);

    joint_msg.position = {0.0,0.0,0.0,0.0,0.0,0.0};
    joint_msg.velocity = {0.0,0.0,0.0,0.0,0.0,0.0};
    tcp_msg.position = {0.001871337890625, -1.145863978938533e-19, -0.06545825958251954, -0.15641778385716565, 1.0340304101706983, -0.2723418753213803};
    tcp_msg.velocity = {0.0,0.0,0.0,0.0,0.0,0.0};

    while (ros::ok())
    {
        joint_msg.header.stamp = ros::Time::now();    // 添加时间戳
        tcp_msg.header.stamp = ros::Time::now();    // 添加时间戳

        pub_joint.publish(joint_msg);
        pub_tcp.publish(tcp_msg);
        // ROS_INFO_STREAM("Published : [" 
        //                 << joint_msg.position[0] << ", "
        //                 << joint_msg.position[1] << ", "
        //                 << joint_msg.position[2] << ", "
        //                 << joint_msg.position[3] << ", "
        //                 << joint_msg.position[4] << ", "
        //                 << joint_msg.position[5] << "]");

        rate.sleep();
    }
    ROS_INFO("begin to pub robot test");

    return 0;
}
