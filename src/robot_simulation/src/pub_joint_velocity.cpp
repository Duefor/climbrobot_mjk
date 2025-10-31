// 发布关节角速度
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

double freq = 10;

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"pub_joint_velocity");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_velocities",10);
    sensor_msgs::JointState msg;
    msg.velocity = {0.5,0.0,0.0,0.0,0.0,0.0};
    ros::Rate rate(freq);
    while (ros::ok())
    {
        pub.publish(msg);
        std::cout << "发布关节角速度：" << '[' << msg.velocity[0] << ',' << msg.velocity[1] << ',' << msg.velocity[2] << ','
            << msg.velocity[3] << ','<< msg.velocity[4] << ','<< msg.velocity[5] << ']' << std::endl;
        rate.sleep();
    }
    
    return 0;
}
