/*
每条消息包含 6 个关节角度
话题名：/joint_states_target
 */

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <cstdlib>
#include <ctime>
#include <sensor_msgs/JointState.h>
#include <iostream>

const double JOINT_MIN[6] = {-1.0, -1.2, -1.5, -2.0, -1.0, -1.5};  // 每个关节的最小角度
const double JOINT_MAX[6] = { 1.0,  1.2,  1.5,  2.0,  1.0,  1.5};  // 每个关节的最大角度
const int PUBLISH_RATE_HZ = 2;   // 发布频率 (Hz)


int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_target_publisher_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    ros::Rate rate(PUBLISH_RATE_HZ);
    std::srand(std::time(nullptr));

    sensor_msgs::JointState msg;
    msg.position.resize(6);
    msg.velocity.resize(6);
    msg.name = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};   // 需要与urdf中关节名称一致

    while (ros::ok())
    {
        msg.header.stamp = ros::Time::now();    // 添加时间戳
        for (int i = 0; i < 6; ++i)
        {
            // 随机生成 [JOINT_MIN[i], JOINT_MAX[i]] 的弧度值
            double val = JOINT_MIN[i] + (std::rand() / (double)RAND_MAX) * (JOINT_MAX[i] - JOINT_MIN[i]);
            msg.position[i] = val;
            // msg.velocity[i] = val;
        }

        pub.publish(msg);
        ROS_INFO_STREAM("Published joint target: [" 
                        << msg.position[0] << ", "
                        << msg.position[1] << ", "
                        << msg.position[2] << ", "
                        << msg.position[3] << ", "
                        << msg.position[4] << ", "
                        << msg.position[5] << "]");

        rate.sleep();
    }

    return 0;
}
