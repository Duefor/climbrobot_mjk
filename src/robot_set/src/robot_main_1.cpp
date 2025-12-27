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
const ELITE::vector6d_t root_joint_pose{0.0,-1.07,-1.97,-1.67,1.57,-1.57};    // joint零点位姿

// sdk中关节角速度最大值，为[150，150，180，230，230，230] （°/s）
const double SDK_MAX_QDOT[6] = {5*M_PI/6, 5*M_PI/6, M_PI, 23*M_PI/18, 23*M_PI/18, 23*M_PI/18};
// 使用手册中最大速度的1/10，这里与速度ik处保持一致
const double MAX_QDOT[6] = {
    SDK_MAX_QDOT[0] / 20.0,
    SDK_MAX_QDOT[1] / 20.0,
    SDK_MAX_QDOT[2] / 20.0,
    SDK_MAX_QDOT[3] / 20.0,
    SDK_MAX_QDOT[4] / 20.0,
    SDK_MAX_QDOT[5] / 20.0
};

const double MAX_QDDOT[6] = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4};   // 关节角加速度限额

// 订阅回调：控制机械臂各关节速度
void jointCallback(const sensor_msgs::JointState::ConstPtr& msg, EliteCSRobotSDK* robot)
{
    if (msg->velocity.size() != 6) return;

    // 当前时间
    ros::Time now = ros::Time::now();

    // 频率 / 时间基准
    static ros::Time last;
    static ELITE::vector6d_t last_qdot = {0,0,0,0,0,0};

    // 第一次进入，建立时间基准，防止 dt 异常
    if (last.isZero())
    {
        last = now;
        for (int i = 0; i < 6; ++i)
            last_qdot[i] = msg->velocity[i];
        return;
    }

    double dt = (now - last).toSec();
    if (dt < 0.02) return;   // 50 Hz
    last = now;

    // 读取期望关节速度
    ELITE::vector6d_t qdot;
    for (int i = 0; i < 6; ++i)
        qdot[i] = msg->velocity[i];

    // 关节速度硬限幅
    for (int i = 0; i < 6; ++i)
    {
        if (qdot[i] >  MAX_QDOT[i]) qdot[i] =  MAX_QDOT[i];
        if (qdot[i] < -MAX_QDOT[i]) qdot[i] = -MAX_QDOT[i];
    }

    // 关节加速度限幅
    for (int i = 0; i < 6; ++i)
    {
        double max_delta = MAX_QDDOT[i] * dt;
        double delta = qdot[i] - last_qdot[i];

        if (delta >  max_delta) delta =  max_delta;
        if (delta < -max_delta) delta = -max_delta;

        qdot[i] = last_qdot[i] + delta;
    }

    // 更新历史速度（只在真正发送指令时）
    last_qdot = qdot;

    robot->jointSpeed(qdot, 0);
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

void tcpStatePublisher(EliteCSRobotSDK* robot, ros::Publisher* pub)
{
    ros::Rate rate(50);  // 发布频率 50Hz
    robot_set::TCPState msg;
    msg.position.resize(6);
    while (ros::ok())
    {
        ELITE::vector6d_t tcpPose = robot->getCurrentTCPPose();
        for (int i = 0; i < 6; i++) msg.position[i] = tcpPose[i];
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

    // 机械臂位姿移动到原点
    cs66robot.moveJoint(root_joint_pose,30.0);
    std::cout << "Robot move to root successful" << std::endl;

    // 订阅关节速度话题
    ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/joint_vel", 1, bind(jointCallback, _1, &cs66robot));
    // 发布关节状态
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::Publisher tcp_pub = nh.advertise<robot_set::TCPState>("/tcp_state", 10);

    // 启动关节状态发布线程
    std::thread pub_joint_thread(jointStatePublisher, &cs66robot, &joint_pub);
    std::thread pub_tcp_thread(tcpStatePublisher, &cs66robot, &tcp_pub);

    ros::waitForShutdown();

    // 断开远程连接
    pub_joint_thread.join();
    pub_tcp_thread.join();
    cs66robot.disconnect();

    return 0;
}
