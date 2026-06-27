#include "cs66_robot_controller.h"
#include <ros/package.h>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

#if defined(__linux) || defined(linux) || defined(__linux__)
#include <sys/mman.h>
#include <pthread.h>
#include <errno.h>
#include <cstring>
#endif


/**
 * @brief Catmull-Rom 样条插值 (位置 XYZ) - 内部静态函数
 */
static double CatmullRom(double p0, double p1, double p2, double p3, double t) {
    double v0 = (p2 - p0) * 0.5;
    double v1 = (p3 - p1) * 0.5;
    double t2 = t * t;
    double t3 = t * t * t;
    return (2 * p1 - 2 * p2 + v0 + v1) * t3 + (-3 * p1 + 3 * p2 - 2 * v0 - v1) * t2 + v0 * t + p1;
}

/**
 * @brief 四元数球面插值 (姿态 RxRyRz) - 内部静态函数
 */
static ELITE::vector6d_t SlerpRotation(
    const ELITE::vector6d_t& start_pose,
    const ELITE::vector6d_t& end_pose, 
    double t) 
{
    tf2::Quaternion q_start, q_end;
    q_start.setRPY(start_pose[3], start_pose[4], start_pose[5]);
    q_end.setRPY(end_pose[3], end_pose[4], end_pose[5]);

    // 使用 tf2 的 slerp 进行平滑插值
    // 限制 t 在 0~1 之间，防止数值计算误差导致的越界
    t = std::max(0.0, std::min(1.0, t));
    tf2::Quaternion q_interp = q_start.slerp(q_end, t);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_interp).getRPY(roll, pitch, yaw);

    // 返回结果只更新姿态部分
    ELITE::vector6d_t result = start_pose; // 拷贝一份
    result[3] = roll;
    result[4] = pitch;
    result[5] = yaw;
    return result;
}

static double LinearInterp(double start, double end, double t) {
    return start + (end - start) * t;
}


CS66RobotController::CS66RobotController(ros::NodeHandle& nh, ros::NodeHandle& private_nh) 
    : nh_(nh),
      private_nh_(private_nh),
      servoj_queue_pre_recv_size_(0),
      servoj_queue_pre_recv_timeout_ms_(0.0f),
      servo_mode_enabled_(false),
      is_initialized_(false),
      is_connected_(false),
      emergency_stop_(false),
      is_move_finish_(false) {
    
    // 从ROS参数服务器读取配置
    loadParameters();
    
    // 初始化ROS接口
    initializeROSInterface();
    
    // 初始化Elite SDK 包括rtsi以及dashboard，并开机、启动、释放抱闸、运行任务
    initializeEliteSDK();
    
    ROS_INFO("CS66 Robot Controller initialized successfully");
}

CS66RobotController::~CS66RobotController() {
    std::cout << "11111111111" << std::endl;
    if (is_connected_) {
        stopRobot();
    }
}

void CS66RobotController::loadParameters() {
    // 从ROS参数服务器读取配置
    private_nh_.param<std::string>("robot_ip", robot_ip_, "192.168.1.199");
    private_nh_.param<std::string>("local_ip", local_ip_, "192.168.1.234");
    private_nh_.param<bool>("headless_mode", headless_mode_, true);
    private_nh_.param<double>("control_frequency", control_frequency_, 50.0);
    private_nh_.param<double>("status_frequency", status_frequency_, 250.0);
    
    // 队列模式参数
    bool servo_mode;
    private_nh_.param<bool>("enable_queue_mode", servo_mode, true);
    servo_mode_enabled_ = servo_mode;
    private_nh_.param<int>("servoj_queue_pre_recv_size", servoj_queue_pre_recv_size_, 20);
    private_nh_.param<float>("servoj_queue_pre_recv_timeout_ms", servoj_queue_pre_recv_timeout_ms_, 0.0);
    
    // 话题名称配置
    private_nh_.param<std::string>("joint_state_topic", joint_state_topic_, "/joint_states");
    private_nh_.param<std::string>("tcp_pose_topic", tcp_pose_topic_, "/tcp_pose");
    private_nh_.param<std::string>("tcp_force_topic", tcp_force_topic_, "/tcp_force");
    // private_nh_.param<std::string>("joint_command_topic", joint_command_topic_, "/joint_command");
    // private_nh_.param<std::string>("tcp_command_topic", tcp_command_topic_, "/tcp_command");

    config_.script_file_path = ros::package::getPath("cs66_ctl_manipulation") + "/source/external_control.script";

    
    // 配置EliteDriverConfig - 确保所有参数都在有效范围内
    config_.robot_ip = robot_ip_;
    config_.local_ip = local_ip_;
    config_.headless_mode = headless_mode_;
    
    // 验证并设置servoj_time（范围：0.002-0.2秒）
    config_.servoj_time = 0.005;

    //设置机械臂检测过程中最大速度max_velocity
    max_velocity = 0.02; // 单位 m/s，根据实际需求调整
    
    // 验证并设置队列参数
    // servoj_queue_pre_recv_size 应该是正整数，范围通常在1-100之间
    config_.servoj_queue_pre_recv_size = 20;
    
    // servoj_queue_pre_recv_timeout 应该是秒（不是毫秒），如果<=0则SDK会自动计算
    // 将毫秒转换为秒，如果值为0或负数，使用-1让SDK自动计算
    if (servoj_queue_pre_recv_timeout_ms_ <= 0) {
        config_.servoj_queue_pre_recv_timeout = -1.0f;  // SDK会自动计算
    } else {
        config_.servoj_queue_pre_recv_timeout = servoj_queue_pre_recv_timeout_ms_ / 1000.0f;  // 转换为秒
    }
    
    // 设置其他默认参数以确保完整性
    config_.servoj_lookahead_time = 0.1f;  // 默认值，范围：0.03-0.2
    config_.servoj_gain = 300;            // 默认值
    config_.stopj_acc = 8.0f;             // 默认值
    
    // 端口配置（使用SDK默认值）
    config_.script_sender_port = 50002;
    config_.reverse_port = 50001;
    config_.trajectory_port = 50003;
    config_.script_command_port = 50004;
    
    ROS_INFO("Parameters loaded - Robot IP: %s, Headless Mode: %s, Queue Mode: %s", 
             robot_ip_.c_str(), headless_mode_ ? "true" : "false", 
             servo_mode_enabled_ ? "true" : "false");
    ROS_INFO("EliteDriverConfig: servoj_time=%f, queue_size=%d, queue_timeout=%f", 
             config_.servoj_time, config_.servoj_queue_pre_recv_size, config_.servoj_queue_pre_recv_timeout);
}

void CS66RobotController::initializeROSInterface() {
    // 初始化发布器
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_state_topic_, 10);
    tcp_pose_pub_ = nh_.advertise<geometry_msgs::Pose>(tcp_pose_topic_, 10);
    tcp_force_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(tcp_force_topic_, 10);
    
    // 初始化定时器  ros的定时器默认创建的时候自动启动
    status_timer_ = nh_.createTimer(ros::Duration(1.0/status_frequency_), 
                                  &CS66RobotController::statusTimerCallback, this);
    
    // 初始化保持连接定时器，每 200ms 发送一次 IDLE 指令（5Hz），防止机器人端脚本超时退出
    keepalive_timer_ = nh_.createTimer(ros::Duration(0.2), 
                                       &CS66RobotController::keepaliveTimerCallback, this);
    
    ROS_INFO("ROS interface initialized");
}

void CS66RobotController::initializeEliteSDK() {
    try {
        // 验证配置参数
        if (config_.robot_ip.empty()) {
            ROS_ERROR("Robot IP is empty!");
            throw std::runtime_error("Robot IP is empty");
        }
        
        if (config_.script_file_path.empty()) {
            ROS_ERROR("Script file path is empty!");
            throw std::runtime_error("Script file path is empty");
        }
        
        // 检查脚本文件是否存在
        std::ifstream script_file(config_.script_file_path);
        if (!script_file.good()) {
            ROS_ERROR("Script file does not exist or is not readable: %s", config_.script_file_path.c_str());
            throw std::runtime_error("Script file not found: " + config_.script_file_path);
        }
        script_file.close();
        
        // 验证数值参数
        if (config_.servoj_time <= 0 || config_.servoj_time > 0.2 || std::isnan(config_.servoj_time) || std::isinf(config_.servoj_time)) {
            ROS_ERROR("Invalid servoj_time: %f", config_.servoj_time);
            throw std::runtime_error("Invalid servoj_time value");
        }
        
        if (config_.servoj_queue_pre_recv_size <= 0 || config_.servoj_queue_pre_recv_size > 1000) {
            ROS_ERROR("Invalid servoj_queue_pre_recv_size: %d", config_.servoj_queue_pre_recv_size);
            throw std::runtime_error("Invalid servoj_queue_pre_recv_size value");
        }
        
        // 创建Elite SDK组件
        driver_ = std::make_unique<EliteDriver>(config_);
        dashboard_ = std::make_unique<DashboardClient>();
        rtsi_client_ = std::make_unique<RtsiIOInterface>(
            ros::package::getPath("cs66_ctl_manipulation") + "/source/output_recipe.txt",
            ros::package::getPath("cs66_ctl_manipulation") + "/source/input_recipe.txt", 
            250);
    
        // 连接dashboard
        if (!dashboard_->connect(config_.robot_ip)) {
            ROS_ERROR("Failed to connect to dashboard");
            throw std::runtime_error("Failed to connect to dashboard");
        }
        
        // 连接rtsi
        if (!rtsi_client_->connect(config_.robot_ip)) {
            ROS_ERROR("Failed to connect to RTSI");
            throw std::runtime_error("Failed to connect to RTSI");
        }
        
        is_connected_ = true;
        
        // 开机
        if (!dashboard_->powerOn()) {
            ROS_ERROR("Power-on failed");
            throw std::runtime_error("Power-on failed");
        }
        
        // 释放抱闸
        if (!dashboard_->brakeRelease()) {
            ROS_ERROR("Brake release failed");
            throw std::runtime_error("Brake release failed");
        }
        
        // 启动外部控制
        if (config_.headless_mode) {
            if (!driver_->isRobotConnected()) {
                if (!driver_->sendExternalControlScript()) {
                    ROS_ERROR("Failed to send external control script");
                    throw std::runtime_error("Failed to send external control script");
                }
            }
        } else {
            if (!dashboard_->playProgram()) {
                ROS_ERROR("Failed to play program");
                throw std::runtime_error("Failed to play program");
            }
        }
        
        // 等待外部控制脚本运行
        while (!driver_->isRobotConnected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        is_initialized_ = true;
        is_move_finish_ = true;  // 初始化完成，设置为空闲状态
        ROS_INFO("Robot initialized successfully");
        
        if(driver_->zeroFTSensor()){
            ROS_INFO("Zero FTSensor successfully");
        } else {
            ROS_ERROR("Zero FTSensor failed");
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize Elite SDK: %s", e.what());
        throw;
    }
}


void CS66RobotController::stopRobot() {
    if (!is_connected_) return;
    
    ROS_INFO("Stopping robot...");
    
    // 发送停止命令
    if (driver_) {
        driver_->writeIdle(0);
        driver_->stopControl();
    }
    // 断开连接
    if (dashboard_) {
        dashboard_->disconnect();
    }
    
    is_connected_ = false;
    is_initialized_ = false;
    
    ROS_INFO("Robot stopped");
}


void CS66RobotController::publishJointStates() {
    if (!is_initialized_ || !rtsi_client_) return;
    
    try {
        // 获取实际关节位置
        vector6d_t actual_joints = rtsi_client_->getActualJointPositions();
        
        // 创建JointState消息
        sensor_msgs::JointState joint_msg;
        joint_msg.header.stamp = ros::Time::now();
        // 使用 MoveIt SRDF 中的关节名称（带下划线）
        joint_msg.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
        joint_msg.position.resize(6);
        
        for (int i = 0; i < 6; i++) {
            joint_msg.position[i] = actual_joints[i];
        }
        
        joint_state_pub_.publish(joint_msg);
        
    } catch (const std::exception& e) {
        ROS_WARN("Failed to get joint positions: %s", e.what());
    }
}

void CS66RobotController::publishTCPPose() {
    if (!is_initialized_ || !rtsi_client_) return;
    
    try {
        // 获取实际TCP位姿: [x, y, z, rx, ry, rz]
        // 其中 rx, ry, rz 为旋转向量（axis * angle）
        vector6d_t actual_pose = rtsi_client_->getAcutalTCPPose();
        
        // 创建Pose消息
        geometry_msgs::Pose pose_msg;
        
        // 位置 (x, y, z)
        pose_msg.position.x = actual_pose[0];
        pose_msg.position.y = actual_pose[1];
        pose_msg.position.z = actual_pose[2];
        
        const double rx = actual_pose[3];
        const double ry = actual_pose[4];
        const double rz = actual_pose[5];
        const double angle = std::sqrt(rx * rx + ry * ry + rz * rz);

        tf2::Quaternion quat;
        if (angle < 1e-12) {
            quat.setValue(0.0, 0.0, 0.0, 1.0);
        } else {
            tf2::Vector3 axis(rx / angle, ry / angle, rz / angle);
            quat.setRotation(axis, angle);
            quat.normalize();
        }

        pose_msg.orientation.x = quat.x();
        pose_msg.orientation.y = quat.y();
        pose_msg.orientation.z = quat.z();
        pose_msg.orientation.w = quat.w();
        
        tcp_pose_pub_.publish(pose_msg);
        
    } catch (const std::exception& e) {
        ROS_WARN("Failed to get TCP pose: %s", e.what());
    }
}

void CS66RobotController::publishTCPForce() {
    if (!is_initialized_ || !rtsi_client_) return;

    try {
        // 获取实际TCP广义力: [fx, fy, fz, tx, ty, tz]
        vector6d_t actual_force = rtsi_client_->getAcutalTCPForce();

        std_msgs::Float64MultiArray force_msg;
        force_msg.data.resize(6);
        for (int i = 0; i < 6; ++i) {
            force_msg.data[i] = actual_force[i];
        }

        tcp_force_pub_.publish(force_msg);
    } catch (const std::exception& e) {
        ROS_WARN("Failed to get TCP force: %s", e.what());
    }
}

//直接将轨迹数据整条sdk执行，需要知道执行时间和路点数量，单位为rad
bool CS66RobotController::RunJointTrajectory(const std::vector<ELITE::vector6d_t>& trajectory, 
    const std::vector<double>& joint_times, int point_number, float blend_radius) {
    
    is_move_finish_ = false;
    
    driver_->setTrajectoryResultCallback([this](ELITE::TrajectoryMotionResult result) {
        if (result == ELITE::TrajectoryMotionResult::SUCCESS) {
            is_move_finish_ = true;
        }
    });
    driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::START, point_number, 500);

    for(int i = 0; i< point_number; i++){
        ELITE::vector6d_t target_joints = {
            trajectory[i][0],
            trajectory[i][1],
            trajectory[i][2],
            trajectory[i][3],
            trajectory[i][4],
            trajectory[i][5]
        };
        driver_->writeTrajectoryPoint(target_joints, joint_times[i], blend_radius, false);        
    }
    while (!is_move_finish_) {
        if (!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
            ROS_ERROR("Joint trajectory execution failed");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    driver_->writeIdle(0);
    
    return true;
    }

bool CS66RobotController::RunPoseTrajectory(const std::vector<ELITE::vector6d_t>& trajectory, 
    const std::vector<double>& pose_times, int point_number, float blend_radius) {
    
    // 重置运动完成标志
    is_move_finish_ = false;
    
    driver_->setTrajectoryResultCallback([this](ELITE::TrajectoryMotionResult result) {
        if (result == ELITE::TrajectoryMotionResult::SUCCESS) {
            is_move_finish_ = true;
        }
    });
    
    driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::START, point_number, 500);
    
    for(int i = 0; i< point_number; i++){
        ELITE::vector6d_t target_pose = {
            trajectory[i][0],
            trajectory[i][1],
            trajectory[i][2],
            trajectory[i][3],
            trajectory[i][4],
            trajectory[i][5]
        };
        driver_->writeTrajectoryPoint(target_pose, pose_times[i], blend_radius, true);
    }

    while (!is_move_finish_) {
        if (!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
            ROS_ERROR("TCP trajectory execution failed");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return true;
}




bool CS66RobotController::RunServojTrajectory(
    const std::vector<ELITE::vector6d_t>& trajectory,
    const std::vector<double>& times)
{
    ROS_INFO("RunServojTrajectory called: raw_points=%zu time_count=%zu initialized=%s driver=%s emergency_stop=%s",
             trajectory.size(),
             times.size(),
             is_initialized_ ? "true" : "false",
             driver_ ? "true" : "false",
             emergency_stop_ ? "true" : "false");

    if (trajectory.size() < 2 || trajectory.size() != times.size()) {
        ROS_ERROR("Invalid trajectory size for Servoj");
        return false;
    }

    if (!is_initialized_ || !driver_) {
        ROS_ERROR("Robot not init");
        return false;
    }

    const double servo_period = config_.servoj_time > 0 ? config_.servoj_time : 0.004;
    double process_speed = 0.2;   // 0.5rad/s    1rad/s = 57.3 deg/s
    std::vector<ELITE::vector6d_t> dense_traj = GenerateDenseTrajectory(trajectory, process_speed, servo_period);
    ROS_INFO("RunServojTrajectory config: servo_period=%.6f process_speed=%.6f dense_points=%zu",
             servo_period, process_speed, dense_traj.size());
    if (dense_traj.empty()) {
        ROS_ERROR("Generated dense trajectory is empty");
        return false;
    }

    ROS_INFO("RunServojTrajectory first dense target: [%.6f %.6f %.6f %.6f %.6f %.6f]",
             dense_traj.front()[0], dense_traj.front()[1], dense_traj.front()[2],
             dense_traj.front()[3], dense_traj.front()[4], dense_traj.front()[5]);
    ROS_INFO("RunServojTrajectory last dense target: [%.6f %.6f %.6f %.6f %.6f %.6f]",
             dense_traj.back()[0], dense_traj.back()[1], dense_traj.back()[2],
             dense_traj.back()[3], dense_traj.back()[4], dense_traj.back()[5]);

    is_move_finish_ = false;
    auto next = std::chrono::steady_clock::now();
    for (size_t i = 0; i < dense_traj.size(); ++i) {
        if (emergency_stop_) {
            ROS_WARN("Emergency stop triggered during servoj execution");
            break;
        }

        if (!driver_->writeServoj(dense_traj[i], 100, false, false)) {
            ROS_ERROR("writeServoj failed at step %zu/%zu target=[%.6f %.6f %.6f %.6f %.6f %.6f]",
                      i,
                      dense_traj.size(),
                      dense_traj[i][0], dense_traj[i][1], dense_traj[i][2],
                      dense_traj[i][3], dense_traj[i][4], dense_traj[i][5]);
            return false;
        }
        if (!ros::ok()) {
        driver_->writeIdle(0); // 发送停止指令
        return false;
        }
        next += std::chrono::microseconds((int64_t)(servo_period * 1e6));
        std::this_thread::sleep_until(next);
    }
    
    driver_->writeIdle(0);

    is_move_finish_ = true;
    return true;
}

std::vector<ELITE::vector6d_t> CS66RobotController::GenerateDenseTrajectory(
    const std::vector<ELITE::vector6d_t>& raw_points,
    double process_speed,
    double servo_period) 
{
    std::vector<ELITE::vector6d_t> dense_trajectory;
    if (raw_points.size() < 2) return dense_trajectory;

    // 遍历每一段 MoveIt 规划出的轨迹 (P_start -> P_end)
    for (size_t i = 0; i < raw_points.size() - 1; ++i) {
        const auto& p_start = raw_points[i];
        const auto& p_end = raw_points[i+1];

        // A. 计算这一段需要多长时间
        // 找出6个关节中，变动最大的那个关节的差值
        double max_joint_diff = 0.0;
        for (int j = 0; j < 6; ++j) {
            max_joint_diff = std::max(max_joint_diff, std::abs(p_end[j] - p_start[j]));
        }

        // 防止原地不动导致除以0
        if (max_joint_diff < 1e-6) {
            continue; 
        }

        // 计算物理时间： 距离 / 速度
        double duration = max_joint_diff / process_speed; 
        
        // 向上取整计算步数 (确保至少走1步)
        int steps = std::max(1, (int)std::ceil(duration / servo_period));

        // B. 生成插值点
        for (int step = 0; step < steps; ++step) {
            double t = (double)step / steps; // 归一化时间 0.0 ~ 1.0

            ELITE::vector6d_t point;
            for (int k = 0; k < 6; ++k) {
                // [核心修正] 对所有6个关节，全部使用线性插值
                point[k] = LinearInterp(p_start[k], p_end[k], t);
            }
            dense_trajectory.push_back(point);
        }
    }
    
    dense_trajectory.push_back(raw_points.back());

    return dense_trajectory;
}
// {
//     std::vector<ELITE::vector6d_t> dense_trajectory;
    
//     if (raw_points.size() < 2) return dense_trajectory;

//     // 1. 预处理：构建用于 Catmull-Rom 的控制点列表
//     // 为了保证曲线穿过所有点，我们在首尾各重复一次端点
//     std::vector<ELITE::vector6d_t> padded_points;
//     padded_points.push_back(raw_points.front()); // 重复起点
//     padded_points.insert(padded_points.end(), raw_points.begin(), raw_points.end());
//     padded_points.push_back(raw_points.back());  // 重复终点

//     // 2. 遍历每一段 (从 P1->P2, P2->P3 ...)
//     // padded_points 现在的结构是: [P0, P0, P1, P2, ... Pn, Pn]
//     // Catmull-Rom 需要 i-1, i, i+1, i+2
//     // 我们实际插值的段是从 index 1 到 size-3
//     for (size_t i = 1; i < padded_points.size() - 2; ++i) {
        
//         const auto& p0 = padded_points[i-1];
//         const auto& p1 = padded_points[i];
//         const auto& p2 = padded_points[i+1];
//         const auto& p3 = padded_points[i+2];

//         // 3. 计算这一段的物理距离 (使用 P1 和 P2 的欧氏距离)
//         double dist = std::sqrt(std::pow(p2[0]-p1[0], 2) + 
//                                 std::pow(p2[1]-p1[1], 2) + 
//                                 std::pow(p2[2]-p1[2], 2));
        
//         // 4. 根据最大速度计算需要多少时间，进而计算需要多少个点
//         // 防止距离为0
//         if (dist < 1e-6) continue; 
        
//         double duration = dist / max_velocity;
//         int steps = std::max(1, (int)(duration / servo_period));

//         // 5. 生成插值点
//         for (int step = 0; step < steps; ++step) {
//             double t = (double)step / steps; // 归一化时间 [0, 1]

//             ELITE::vector6d_t point;
            
//             // 位置：Catmull-Rom 样条
//             point[0] = CatmullRom(p0[0], p1[0], p2[0], p3[0], t);
//             point[1] = CatmullRom(p0[1], p1[1], p2[1], p3[1], t);
//             point[2] = CatmullRom(p0[2], p1[2], p2[2], p3[2], t);

//             // 姿态使用 p1 和 p2 的球面插值
//             ELITE::vector6d_t ori = SlerpRotation(p1, p2, t);
//             point[3] = ori[3];
//             point[4] = ori[4];
//             point[5] = ori[5];

//             dense_trajectory.push_back(point);
//         }
//     }
//     // 确保把最后一个点加进去
//     dense_trajectory.push_back(raw_points.back());

//     return dense_trajectory;
// }

// 基于时间的密集轨迹生成：使用 raw_points 和对应的时间 points (seconds)
std::vector<ELITE::vector6d_t> CS66RobotController::GenerateDenseTrajectoryTimed(
    const std::vector<ELITE::vector6d_t>& raw_points,
    const std::vector<double>& point_times,
    double servo_period)
{
    std::vector<ELITE::vector6d_t> dense_trajectory;
    if (raw_points.size() < 2 || point_times.size() != raw_points.size() || servo_period <= 0) return dense_trajectory;

    // 与无时间版本相同的 padding 逻辑
    std::vector<ELITE::vector6d_t> padded_points;
    padded_points.push_back(raw_points.front());
    padded_points.insert(padded_points.end(), raw_points.begin(), raw_points.end());
    padded_points.push_back(raw_points.back());

    // padded_points indices: [P0, P0, P1, P2, ... Pn, Pn]
    for (size_t i = 1; i < padded_points.size() - 2; ++i) {
        const auto& p0 = padded_points[i - 1];
        const auto& p1 = padded_points[i];
        const auto& p2 = padded_points[i + 1];
        const auto& p3 = padded_points[i + 2];

        // segment index in raw_points for p1->p2 is (i-1)
        size_t seg_idx = i - 1;
        double dist = std::sqrt(std::pow(p2[0] - p1[0], 2) +
                                std::pow(p2[1] - p1[1], 2) +
                                std::pow(p2[2] - p1[2], 2));

        double duration = 0.0;
        if (seg_idx + 1 < point_times.size()) {
            duration = point_times[seg_idx + 1] - point_times[seg_idx];
        }

        // 如果给定时间无效（<=0），按距离分配最少一个 step
        int steps = 1;
        if (duration > 1e-8) {
            steps = std::max(1, (int)std::ceil(duration / servo_period));
        } else if (dist > 1e-6) {
            // fallback: 根据距离使用一个较小的步数
            steps = std::max(1, (int)std::ceil(dist / (servo_period)));
        }

        for (int step = 0; step < steps; ++step) {
            double t = (double)step / (double)steps; // [0,1)

            ELITE::vector6d_t point;
            point[0] = CatmullRom(p0[0], p1[0], p2[0], p3[0], t);
            point[1] = CatmullRom(p0[1], p1[1], p2[1], p3[1], t);
            point[2] = CatmullRom(p0[2], p1[2], p2[2], p3[2], t);

            // 姿态使用 p1 和 p2 的球面插值
            ELITE::vector6d_t ori = SlerpRotation(p1, p2, t);
            point[3] = ori[3];
            point[4] = ori[4];
            point[5] = ori[5];

            dense_trajectory.push_back(point);
        }
    }

    // 确保末尾点在
    dense_trajectory.push_back(raw_points.back());
    return dense_trajectory;
}

bool CS66RobotController::SendScriptToRobot(const std::string& script) {
    if (!is_initialized_ || !driver_) {
        return false;
    }
    if (!driver_->sendScript(script)) {
        ROS_ERROR("Failed to send script to robot");
        return false;
    }
    return true;
}

// Wrapper: start force mode using EliteDriver with safety checks and logging
bool CS66RobotController::startForceMode(const ELITE::vector6d_t& ref_frame,
                                         const ELITE::vector6int32_t& selection_vector,
                                         const ELITE::vector6d_t& wrench,
                                         const ELITE::vector6d_t& limits)
{
    if (!is_initialized_ || !driver_) {
        ROS_ERROR("startForceMode: Robot not initialized or driver missing");
        return false;
    }
    
    ROS_INFO("Starting Force Mode with wrench = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
             wrench[0], wrench[1], wrench[2], wrench[3], wrench[4], wrench[5]);

    if (!driver_->zeroFTSensor()) {
        ROS_ERROR("Failed to zero force sensor");
        return false;
    }
    int mode = 3; // 参考SDK: 模式3（TCP模式）
    bool ok = false;
    try {
        ok = driver_->startForceMode(ref_frame, selection_vector, wrench, static_cast<ELITE::ForceMode>(mode), limits);
    } catch (const std::exception& e) {
        ROS_ERROR("startForceMode: exception from driver: %s", e.what());
        return false;
    }

    if (!ok) {
        ROS_ERROR("startForceMode: driver reported failure");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ROS_INFO("startForceMode: started successfully");
    return true;
}


// Wrapper: end force mode
bool CS66RobotController::endForceMode()
{
    if (!is_initialized_ || !driver_) {
        ROS_ERROR("endForceMode: Robot not initialized or driver missing");
        return false;
    }

    bool ok = false;
    try {
        ok = driver_->endForceMode();
    } catch (const std::exception& e) {
        ROS_ERROR("endForceMode: exception from driver: %s", e.what());
        return false;
    }

    if (!ok) {
        ROS_ERROR("endForceMode: driver reported failure");
        return false;
    }

    ROS_INFO("endForceMode: ended successfully");
    return true;
}

void CS66RobotController::statusTimerCallback(const ros::TimerEvent&) {
    publishJointStates();
    publishTCPPose();
    publishTCPForce();
}

void CS66RobotController::keepaliveTimerCallback(const ros::TimerEvent&) {
    // 定期发送 IDLE 指令保持连接，防止机器人端 external_control 脚本因超时退出
    // 机器人端脚本需要持续从 reverse_socket 读取数据，如果长时间没有数据会超时退出并关闭连接
    // 重要：只在空闲状态（is_move_finish_ == true）时发送 IDLE，避免干扰正在执行的轨迹
    if (is_initialized_ && driver_ && driver_->isRobotConnected() && is_move_finish_) {
        driver_->writeIdle(0);
    }
}

// // ==========================================
// // 辅助函数：Catmull-Rom 样条插值 (位置 XYZ)
// // ==========================================
// // P0, P1, P2, P3 是连续的四个路点，我们计算 P1 和 P2 之间的点
// // t 是 P1 到 P2 之间的归一化时间 [0, 1]
// double CatmullRom(double p0, double p1, double p2, double p3, double t) {
//     double v0 = (p2 - p0) * 0.5;
//     double v1 = (p3 - p1) * 0.5;
//     double t2 = t * t;
//     double t3 = t * t * t;
//     return (2 * p1 - 2 * p2 + v0 + v1) * t3 + (-3 * p1 + 3 * p2 - 2 * v0 - v1) * t2 + v0 * t + p1;
// }

// // ==========================================
// // 辅助函数：四元数球面插值 (姿态 RxRyRz)
// // ==========================================
// // start_rpy, end_rpy: [Rx, Ry, Rz] 单位 rad
// ELITE::vector6d_t SlerpRotation(
//     const ELITE::vector6d_t& start_pose,
//     const ELITE::vector6d_t& end_pose, 
//     double t) {
//     tf2::Quaternion q_start, q_end;
//     q_start.setRPY(start_pose[3], start_pose[4], start_pose[5]);
//     q_end.setRPY(end_pose[3], end_pose[4], end_pose[5]);

//     // 使用 tf2 的 slerp 进行平滑插值
//     tf2::Quaternion q_interp = q_start.slerp(q_end, t);
    
//     double roll, pitch, yaw;
//     tf2::Matrix3x3(q_interp).getRPY(roll, pitch, yaw);

//     // 返回结果只更新姿态部分
//     ELITE::vector6d_t result = start_pose; // 拷贝一份
//     result[3] = roll;
//     result[4] = pitch;
//     result[5] = yaw;
//     return result;
// }



