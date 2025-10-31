#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

double freq = 10;

// 全局变量存储关节状态
sensor_msgs::JointState joint_state;

ros::Time last_time;

// 速度回调函数
void velocityCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // 计算时间间隔 dt
    // ros::Time current_time = ros::Time::now();
    // double dt = (current_time - last_time).toSec();
    // last_time = current_time;
    
    double dt = (msg->header.stamp - last_time).toSec();
    if (dt <= 0) return;
    last_time = msg->header.stamp;
    
    // 积分：角度 = 角度 + 速度 * dt
    joint_state.position[0] += msg->velocity[0] * dt;  // joint1
    joint_state.position[1] += msg->velocity[1] * dt;  // joint2
    joint_state.position[2] += msg->velocity[2] * dt;  // joint3
    joint_state.position[3] += msg->velocity[3] * dt; // joint4
    joint_state.position[4] += msg->velocity[4] * dt; // joint5
    joint_state.position[5] += msg->velocity[5] * dt; // joint6

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity_to_position");
    ros::NodeHandle nh;

    joint_state.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    joint_state.position = {0.0, -0.93, -2.21, -1.58, 1.42, 0.0};

    // 初始化时间
    last_time = ros::Time::now();

    // 订阅关节速度指令
    ros::Subscriber vel_sub = nh.subscribe("/joint_vel", 10, velocityCallback);

    // 发布关节角度
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    ros::Rate rate(freq);

    while (ros::ok())
    {
        joint_state.header.stamp = ros::Time::now();
        joint_pub.publish(joint_state);
    
        // std::cout << "关节角速度积分得角度：" << '[' << joint_state.position[0] << ',' << joint_state.position[1] << ',' <<
        //     joint_state.position[2] << ',' <<joint_state.position[3] << ',' <<joint_state.position[4] << ',' <<joint_state.position[5] << ']'
        //     << std::endl;

        rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}