#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_set/TCPState.h>

#include <robot_sdk_wrapper/robot_sdk.h>

#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <cmath>

/* =================== 配置 =================== */

const std::string DEFAULT_ROBOT_IP = "192.168.1.200";
const std::string DEFAULT_PC_IP    = "192.168.1.150";
const std::string external_control_file_address = "/home/barry/workspace/climbrobot_mjk/src/robot_sdk_wrapper/resource/external_control.script";
const std::string output_recipe_file_address    = "/home/barry/workspace/climbrobot_mjk/src/robot_sdk_wrapper/resource/output_recipe.txt";
const std::string input_recipe_file_address     = "/home/barry/workspace/climbrobot_mjk/src/robot_sdk_wrapper/resource/input_recipe.txt";
const std::string task_file_address              = "mjktest.task";

const ELITE::vector6d_t root_joint_pose{0.0,-1.18,-2.44,-1.1,1.57,-1.57};

// SDK 关节最大速度（rad/s）
const double SDK_MAX_QDOT[6] = {
    5*M_PI/6, 5*M_PI/6, M_PI,
    23*M_PI/18, 23*M_PI/18, 23*M_PI/18
};

// 实际使用速度（1/20）
const double MAX_QDOT[6] = {
    SDK_MAX_QDOT[0]/20.0, SDK_MAX_QDOT[1]/20.0, SDK_MAX_QDOT[2]/20.0,
    SDK_MAX_QDOT[3]/20.0, SDK_MAX_QDOT[4]/20.0, SDK_MAX_QDOT[5]/20.0
};

// 最大关节加速度
const double MAX_QDDOT[6] = {0.4,0.4,0.4,0.4,0.4,0.4};

/* =================== 共享数据 =================== */

// 控制指令（callback → SDK IO）
std::atomic<bool> qdot_valid{false};
ELITE::vector6d_t qdot_cmd{0,0,0,0,0,0};

// 状态缓存（SDK IO → 发布线程）
ELITE::vector6d_t joint_cache{0,0,0,0,0,0};
ELITE::vector6d_t tcp_cache{0,0,0,0,0,0};
std::mutex state_mtx;

/* =================== ROS Callback =================== */

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (msg->velocity.size() != 6) return;

    static ros::Time last;
    static ELITE::vector6d_t last_qdot{0,0,0,0,0,0};

    ros::Time now = ros::Time::now();
    if (last.isZero()) { last = now; return; }

    double dt = (now - last).toSec();
    if (dt < 0.02) return;   // 50 Hz
    last = now;

    ELITE::vector6d_t qdot;

    /* 速度限幅 */
    for (int i = 0; i < 6; ++i)
    {
        qdot[i] = msg->velocity[i];
        if (qdot[i] >  MAX_QDOT[i]) qdot[i] =  MAX_QDOT[i];
        if (qdot[i] < -MAX_QDOT[i]) qdot[i] = -MAX_QDOT[i];
    }

    /* 加速度限幅 */
    for (int i = 0; i < 6; ++i)
    {
        double max_delta = MAX_QDDOT[i] * dt;
        double delta = qdot[i] - last_qdot[i];

        if (delta >  max_delta) delta =  max_delta;
        if (delta < -max_delta) delta = -max_delta;

        qdot[i] = last_qdot[i] + delta;
    }

    last_qdot = qdot;

    qdot_cmd = qdot;
    qdot_valid.store(true, std::memory_order_release);
}

/* =================== SDK IO 线程 =================== */

void sdkIOThread(EliteCSRobotSDK* robot)
{
    ros::Rate rate(250);   // SDK 控制周期

    while (ros::ok())
    {
        /* 下发控制 */
        if (qdot_valid.load(std::memory_order_acquire))
        {
            robot->jointSpeed(qdot_cmd, 0);
        }

        /* 读取状态 */
        ELITE::vector6d_t joints = robot->getCurrentJoint();
        ELITE::vector6d_t tcp    = robot->getCurrentTCPPose();

        {
            std::lock_guard<std::mutex> lock(state_mtx);
            joint_cache = joints;
            tcp_cache   = tcp;
        }

        rate.sleep();
    }
}

/* =================== 状态发布线程 =================== */

void jointStatePublisher(ros::Publisher* pub)
{
    ros::Rate rate(50);
    sensor_msgs::JointState msg;
    msg.name = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
    msg.position.resize(6);

    while (ros::ok())
    {
        {
            std::lock_guard<std::mutex> lock(state_mtx);
            for (int i = 0; i < 6; ++i)
                msg.position[i] = joint_cache[i];
        }

        msg.header.stamp = ros::Time::now();
        pub->publish(msg);
        rate.sleep();
    }
}

void tcpStatePublisher(ros::Publisher* pub)
{
    ros::Rate rate(50);
    robot_set::TCPState msg;
    msg.position.resize(6);

    while (ros::ok())
    {
        {
            std::lock_guard<std::mutex> lock(state_mtx);
            for (int i = 0; i < 6; ++i)
                msg.position[i] = tcp_cache[i];
        }

        msg.header.stamp = ros::Time::now();
        pub->publish(msg);
        rate.sleep();
    }
}

/* =================== main =================== */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_rt_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle nh;

    EliteCSRobotSDK cs66robot(
        DEFAULT_ROBOT_IP,
        DEFAULT_PC_IP,
        true,
        external_control_file_address,
        output_recipe_file_address,
        input_recipe_file_address,
        task_file_address,
        250
    );

    if (!cs66robot.init() || !cs66robot.start())
    {
        std::cerr << "Robot init/start failed" << std::endl;
        return 1;
    }

    cs66robot.moveJoint(root_joint_pose, 30.0);

    ros::Subscriber sub =
        nh.subscribe<sensor_msgs::JointState>("/joint_vel", 1, jointCallback);

    ros::Publisher joint_pub =
        nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    ros::Publisher tcp_pub =
        nh.advertise<robot_set::TCPState>("/tcp_state", 10);

    std::thread sdk_thread(sdkIOThread, &cs66robot);
    std::thread pub_joint_thread(jointStatePublisher, &joint_pub);
    std::thread pub_tcp_thread(tcpStatePublisher, &tcp_pub);

    ros::waitForShutdown();

    sdk_thread.join();
    pub_joint_thread.join();
    pub_tcp_thread.join();
    cs66robot.disconnect();

    return 0;
}
