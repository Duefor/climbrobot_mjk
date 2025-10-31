// 订阅笛卡尔速度并控制
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
const std::string external_control_file_address = "/home/barry/workspace/climbrobot_mjk/src/robot_sdk_wrapper/resource/external_control.script";
const std::string output_recipe_file_address = "/home/barry/workspace/climbrobot_mjk/src/robot_sdk_wrapper/resource/output_recipe.txt";
const std::string input_recipe_file_address = "/home/barry/workspace/climbrobot_mjk/src/robot_sdk_wrapper/resource/input_recipe.txt";
const std::string task_file_address = "mjktest.task";


// 订阅回调：控制机械臂笛卡尔速度
void TCPCallback(const robot_set::TCPState::ConstPtr& msg, EliteCSRobotSDK* robot)
{
    if(msg->velocity.size()!=6) return;
    ELITE::vector6d_t cartesian_speed;
    for(int i=0;i<6;i++)
    {
        cartesian_speed[i] = msg->velocity[i];
    }
    // // 限制sdk的调用频率，避免超过控制接口带宽
    // static ros::Time last;
    // if ((ros::Time::now() - last).toSec() < 0.02) return;  // 50 Hz
    // last = ros::Time::now();

    // 可以尝试限制笛卡尔速度，（后续实现）
    robot->lineSpeed(cartesian_speed,0);
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

// 发布笛卡尔位姿信息
void tcpStatePublisher(EliteCSRobotSDK* robot, ros::Publisher* pub)
{
    ros::Rate rate(50);  // 50 Hz
    robot_set::TCPState msg;
    msg.position.resize(6);
    while (ros::ok())
    {
        ELITE::vector6d_t tcp = robot->getCurrentTCPPose();  // [x, y, z, rx, ry, rz]
        for (int i = 0; i < 6; i++) msg.position[i] = tcp[i];
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

    ELITE::vector6d_t startline = {0.3 ,0 ,0.3, 3.14, 0, 3.14};
    cs66robot.moveLine(startline,5);
    std::cout << "Robot move to start successful" << std::endl;


    // 订阅笛卡尔速度话题
    ros::Subscriber sub = nh.subscribe<robot_set::TCPState>("/cartesian_vel", 1, bind(TCPCallback, _1, &cs66robot));
    // 发布关节状态
    ros::Publisher joints_pub = nh.advertise<sensor_msgs::JointState>("/joints_state", 10);
    // 发布笛卡尔位姿
    ros::Publisher tcp_pub = nh.advertise<robot_set::TCPState>("/tcp_state", 10);

    // 启动关节状态发布线程
    std::thread pub_thread(jointStatePublisher, &cs66robot, &joints_pub);
    // 启动笛卡尔位姿线程
    std::thread tcp_thread(tcpStatePublisher, &cs66robot, &tcp_pub);

    ros::waitForShutdown();

    // 断开远程连接
    if(pub_thread.joinable()) pub_thread.join();
    if(tcp_thread.joinable()) tcp_thread.join();
    cs66robot.disconnect();

    return 0;
}
