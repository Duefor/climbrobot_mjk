// // 订阅关节速度并控制
// // 发布实时关节角度
// // sdk调用未加锁
// #include <ros/ros.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <vector>
// #include <sstream>
// #include <robot_set/TCPState.h>
// #include <sensor_msgs/JointState.h>

// #include <robot_sdk_wrapper/robot_sdk.h>

// #include <memory>
// #include <thread>
// #include <iostream>


// const std::string DEFAULT_ROBOT_IP = "192.168.1.200";
// const std::string DEFAULT_PC_IP = "192.168.1.150";
// const std::string external_control_file_address = "external_control.script";
// const std::string output_recipe_file_address = "output_recipe.txt";
// const std::string input_recipe_file_address = "input_recipe.txt";
// const std::string task_file_address = "mjktest.task";
// const ELITE::vector6d_t root_joint_pose{0.0,-1.18,-2.44,-1.1,1.57,-1.57};    // joint零点位姿

// // sdk中关节角速度最大值，为[150，150，180，230，230，230] （°/s）
// const double SDK_MAX_QDOT[6] = {5*M_PI/6, 5*M_PI/6, M_PI, 23*M_PI/18, 23*M_PI/18, 23*M_PI/18};
// // 使用手册中最大速度的1/10，这里与速度ik处保持一致
// const double MAX_QDOT[6] = {
//     SDK_MAX_QDOT[0] / 20.0,
//     SDK_MAX_QDOT[1] / 20.0,
//     SDK_MAX_QDOT[2] / 20.0,
//     SDK_MAX_QDOT[3] / 20.0,
//     SDK_MAX_QDOT[4] / 20.0,
//     SDK_MAX_QDOT[5] / 20.0
// };

// const double MAX_QDDOT[6] = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4};   // 关节角加速度限额

// // 订阅回调：控制机械臂各关节速度
// void jointCallback(const sensor_msgs::JointState::ConstPtr& msg, EliteCSRobotSDK* robot)
// {
//     if (msg->velocity.size() != 6) return;

//     // 当前时间
//     ros::Time now = ros::Time::now();

//     // 频率 / 时间基准
//     static ros::Time last;
//     static ELITE::vector6d_t last_qdot = {0,0,0,0,0,0};

//     // 第一次进入，建立时间基准，防止 dt 异常
//     if (last.isZero())
//     {
//         last = now;
//         for (int i = 0; i < 6; ++i)
//             last_qdot[i] = msg->velocity[i];
//         return;
//     }

//     double dt = (now - last).toSec();
//     if (dt < 0.02) return;   // 50 Hz
//     last = now;

//     // 读取期望关节速度
//     ELITE::vector6d_t qdot;
//     for (int i = 0; i < 6; ++i)
//         qdot[i] = msg->velocity[i];

//     // 关节速度硬限幅
//     for (int i = 0; i < 6; ++i)
//     {
//         if (qdot[i] >  MAX_QDOT[i]) qdot[i] =  MAX_QDOT[i];
//         if (qdot[i] < -MAX_QDOT[i]) qdot[i] = -MAX_QDOT[i];
//     }

//     // 关节加速度限幅
//     for (int i = 0; i < 6; ++i)
//     {
//         double max_delta = MAX_QDDOT[i] * dt;
//         double delta = qdot[i] - last_qdot[i];

//         if (delta >  max_delta) delta =  max_delta;
//         if (delta < -max_delta) delta = -max_delta;

//         qdot[i] = last_qdot[i] + delta;
//     }

//     // 更新历史速度（只在真正发送指令时）
//     last_qdot = qdot;

//     robot->jointSpeed(qdot, 0);
// }

// // 发布关节角度信息
// void jointStatePublisher(EliteCSRobotSDK* robot, ros::Publisher* pub)
// {
//     ros::Rate rate(50);  // 发布频率 50Hz
//     sensor_msgs::JointState msg;
//     msg.position.resize(6);
//     msg.name = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
//     while (ros::ok())
//     {
//         ELITE::vector6d_t joints = robot->getCurrentJoint();
//         for (int i = 0; i < 6; i++) msg.position[i] = joints[i];
//         msg.header.stamp = ros::Time::now();
//         pub->publish(msg);
        
//         rate.sleep();
//     }
// }

// void tcpStatePublisher(EliteCSRobotSDK* robot, ros::Publisher* pub)
// {
//     ros::Rate rate(50);  // 发布频率 50Hz
//     robot_set::TCPState msg;
//     msg.position.resize(6);
//     while (ros::ok())
//     {
//         ELITE::vector6d_t tcpPose = robot->getCurrentTCPPose();
//         for (int i = 0; i < 6; i++) msg.position[i] = tcpPose[i];
//         msg.header.stamp = ros::Time::now();
//         pub->publish(msg);
        
//         rate.sleep();
//     }
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "main");
//     ros::AsyncSpinner spinner(2);
//     spinner.start();
//     ros::NodeHandle nh;

//     EliteCSRobotSDK cs66robot(DEFAULT_ROBOT_IP,DEFAULT_PC_IP,external_control_file_address,
//                     output_recipe_file_address,input_recipe_file_address,task_file_address,250);

//     if(!cs66robot.init()){
//         std::cout << "Robot init false" << std::endl;
//         return 1;
//     }
//     std::cout << "Robot init successful" << std::endl;
//     if(!cs66robot.start()){
//         std::cout << "Robot start false" << std::endl;
//         return 1;
//     }
//     std::cout << "Robot start successful" << std::endl;

//     // 机械臂位姿移动到原点
//     cs66robot.moveJoint(root_joint_pose,30.0);
//     std::cout << "Robot move to root successful" << std::endl;

//     // 订阅关节速度话题
//     ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/joint_vel", 1, bind(jointCallback, _1, &cs66robot));
//     // 发布关节状态
//     ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
//     ros::Publisher tcp_pub = nh.advertise<robot_set::TCPState>("/tcp_state", 10);

//     // 启动关节状态发布线程
//     std::thread pub_joint_thread(jointStatePublisher, &cs66robot, &joint_pub);
//     std::thread pub_tcp_thread(tcpStatePublisher, &cs66robot, &tcp_pub);

//     ros::waitForShutdown();

//     // 断开远程连接
//     pub_joint_thread.join();
//     pub_tcp_thread.join();
//     cs66robot.disconnect();

//     return 0;
// }


// 完善版本，sdk调用加锁
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_set/TCPState.h>

#include <robot_sdk_wrapper/robot_sdk.h>

#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <cmath>

// 配置
const double robot_pub_rate = 100; // 机械臂发布状态频率，注意不能大于sdk带宽，最好小于带宽的1/2
const double robot_sdk_rate = 250; // 机械臂sdk调用频率

std::string DEFAULT_ROBOT_IP;   // 机械臂ip
std::string DEFAULT_PC_IP;  // PCip
std::string external_control_file_address;  // 外部控制文件
std::string output_recipe_file_address; // 外部控制文件
std::string input_recipe_file_address;  // 外部控制文件
std::string task_file_address;    // 机械臂配置/任务文件名

const ELITE::vector6d_t root_joint_pose{0.0,-1.18,-2.44,-1.1,1.57,-1.57};   // 关节初始位置

const ELITE::vector6d_t joint_zero_speed{0.0,0.0,0.0,0.0,0.0,0.0};  // 关节速度置零

// SDK 关节最大速度（rad/s）
const double SDK_MAX_QDOT[6] = {
    5*M_PI/6, 5*M_PI/6, M_PI,
    23*M_PI/18, 23*M_PI/18, 23*M_PI/18
};

// 实际使用速度（1/20）
const double MAX_QDOT[6] = {
    SDK_MAX_QDOT[0]/5.0, 
    SDK_MAX_QDOT[1]/5.0, 
    SDK_MAX_QDOT[2]/5.0,
    SDK_MAX_QDOT[3]/5.0, 
    SDK_MAX_QDOT[4]/5.0, 
    SDK_MAX_QDOT[5]/5.0
};

// 最大关节加速度（比ik稍微大一点）
const double MAX_QDDOT[6] ={1.2,
                            1.2,
                            1.2,
                            1.2,
                            1.2,
                            1.2};

// 安全保护
std::atomic<double> last_cmd_time{0.0};
// 机械臂sdk调用超时
const double CMD_TIMEOUT = 0.05;   // 50 ms（建议 < 2 * IK 频率）



// 控制指令（callback → SDK IO）
std::atomic<bool> qdot_valid{false};
ELITE::vector6d_t qdot_cmd{0,0,0,0,0,0};

// 状态缓存（SDK IO → 发布线程）
ELITE::vector6d_t joint_cache{0,0,0,0,0,0};
ELITE::vector6d_t tcp_cache{0,0,0,0,0,0};
std::mutex state_mtx;



void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (msg->velocity.size() != 6) return;

    static ros::Time last;
    static ELITE::vector6d_t last_qdot{0,0,0,0,0,0};

    ros::Time now = ros::Time::now();
    if (last.isZero()) { last = now; return; }

    double dt = (now - last).toSec();

    // 不能接收太快的信息
    if (dt < 0.02) return;   // 50 Hz
    last = now;

    // 关节速度发布太慢或是断线重连
    if (dt > 0.1)
    {
        // 认为 IK 刚恢复，重置加速度历史
        for (int i = 0; i < 6; ++i) last_qdot[i] = msg->velocity[i];
        qdot_valid.store(false, std::memory_order_release);
        return;
    }

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
    // 超时保护
    qdot_valid.store(true, std::memory_order_release);
    last_cmd_time.store(ros::Time::now().toSec(), std::memory_order_release);
    
}

// 机械臂sdk线程

void sdkIOThread(EliteCSRobotSDK* robot)
{
    ros::Rate rate(robot_sdk_rate);   // SDK 控制周期

    while (ros::ok())
    {
        double now = ros::Time::now().toSec();
        double last = last_cmd_time.load(std::memory_order_acquire);
        /* 下发控制 */
        if (qdot_valid.load(std::memory_order_acquire) && (now - last) < CMD_TIMEOUT)
        {
            robot->jointSpeed(qdot_cmd, 0);
        }
        else
        {
            // 安全保护
            robot->jointSpeed(joint_zero_speed, 0);
            qdot_valid.store(false, std::memory_order_release);
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



void jointStatePublisher(ros::Publisher* pub)
{
    ros::Rate rate(robot_pub_rate);
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
    ros::Rate rate(robot_pub_rate);
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_main_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle nh;

    ros::param::param(std::string("~DEFAULT_ROBOT_IP"), DEFAULT_ROBOT_IP, std::string("192.168.1.199"));
    ros::param::param(std::string("~DEFAULT_PC_IP"), DEFAULT_PC_IP, std::string("192.168.1.150"));
    ros::param::param(std::string("~external_control_file_address"), external_control_file_address, std::string("external_control.script"));
    ros::param::param(std::string("~output_recipe_file_address"), output_recipe_file_address, std::string("output_recipe.txt"));
    ros::param::param(std::string("~input_recipe_file_address"), input_recipe_file_address, std::string("input_recipe.txt"));
    ros::param::param(std::string("~task_file_address"), task_file_address, std::string("mjktest.task"));
    

    EliteCSRobotSDK cs66robot(
        DEFAULT_ROBOT_IP,
        DEFAULT_PC_IP,
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

    // 机械臂移动到初始位置
    double arrive_time = 0.0;
    double max_joint = 0.0;
    ELITE::vector6d_t current_joint_pose = cs66robot.getCurrentJoint();
    for(int i = 0; i < 6; i++)
    {
        max_joint = std::max(abs(root_joint_pose[i] - current_joint_pose[i]) , max_joint);
    }
    arrive_time = std::max(max_joint / MAX_QDOT[0], 3.0);
    cs66robot.moveJoint(root_joint_pose, arrive_time);
    std::cout << "机械臂已移动到初始位置" << std::endl;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/velocity_ik/joint_vel", 1, jointCallback);

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/cs66/joint_states", 10);

    ros::Publisher tcp_pub = nh.advertise<robot_set::TCPState>("/cs66/tcp_state", 10);

    std::thread sdk_thread(sdkIOThread, &cs66robot);
    std::thread pub_joint_thread(jointStatePublisher, &joint_pub);
    std::thread pub_tcp_thread(tcpStatePublisher, &tcp_pub);

    ros::waitForShutdown();
    std::cout << "已断开机械臂控制" << std::endl;

    sdk_thread.join();
    pub_joint_thread.join();
    pub_tcp_thread.join();

    cs66robot.jointSpeed(joint_zero_speed,0);
    cs66robot.disconnect();

    return 0;
}
