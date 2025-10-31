// 订阅关节速度并控制
// 发布实时关节角度
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <sstream>
#include <robot_set/TCPState.h>
#include <sensor_msgs/JointState.h>

#include <robot_sdk_wrapper/robot_sdk.h>

#include <memory>
#include <thread>
#include <iostream>


const std::string DEFAULT_ROBOT_IP = "192.168.1.200";
const std::string DEFAULT_PC_IP = "192.168.1.150";
const std::string external_control_file_address = "external_control.script";
const std::string output_recipe_file_address = "output_recipe.txt";
const std::string input_recipe_file_address = "input_recipe.txt";
const std::string task_file_address = "mjktest.task";


// 订阅回调：控制机械臂各关节速度
void jointCallback(const sensor_msgs::JointState::ConstPtr& msg, EliteCSRobotSDK* robot)
{
    if(msg->velocity.size()!=6) return;
    ELITE::vector6d_t joint_speed;
    for(int i=0;i<6;i++)
    {
        joint_speed[i] = msg->velocity[i];
    }
    // 限制sdk的调用频率，避免超过控制接口带宽
    static ros::Time last;
    if ((ros::Time::now() - last).toSec() < 0.02) return;  // 50 Hz
    last = ros::Time::now();

    robot->jointSpeed(joint_speed,0);
}

// 发布关节角度信息
void jointStatePublisher(EliteCSRobotSDK* robot, ros::Publisher* pub)
{
    ros::Rate rate(50);  // 发布频率 50Hz
    sensor_msgs::JointState msg;
    msg.position.resize(6);
    msg.name = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
    while (ros::ok())
    {
        ELITE::vector6d_t joints = robot->getCurrentJoint();
        for (int i = 0; i < 6; i++) msg.position[i] = joints[i];
        msg.header.stamp = ros::Time::now();
        pub->publish(msg);
        
        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle nh;

    EliteCSRobotSDK cs66robot(DEFAULT_ROBOT_IP,DEFAULT_PC_IP,external_control_file_address,
                    output_recipe_file_address,input_recipe_file_address,task_file_address,250);

    if(!cs66robot.init()){
        std::cout << "Robot init false" << std::endl;
        return 1;
    }
    std::cout << "Robot init successful" << std::endl;
    if(!cs66robot.start()){
        std::cout << "Robot start false" << std::endl;
        return 1;
    }
    std::cout << "Robot start successful" << std::endl;

    // 订阅关节速度话题
    ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/joint_vel", 1, bind(jointCallback, _1, &cs66robot));
    // 发布关节状态
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_state", 10);

    // 启动关节状态发布线程
    std::thread pub_thread(jointStatePublisher, &cs66robot, &joint_pub);

    ros::waitForShutdown();

    // 断开远程连接
    pub_thread.join();
    cs66robot.disconnect();

    return 0;
}
