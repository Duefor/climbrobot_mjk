#include "robot_sdk_wrapper/robot_sdk.h"
#include <iostream>

// 构造函数
EliteCSRobotSDK::EliteCSRobotSDK(const std::string& robot_ip, const std::string& pc_ip, const std::string& external_control_script,
                         const std::string& output_recipe, const std::string& input_recipe, const std::string& task_file ,double frequency) :
        robot_ip(robot_ip), pc_ip(pc_ip), external_control_script(external_control_script),
        output_recipe(output_recipe), input_recipe(input_recipe), task_file(task_file), frequency(frequency) {
    // 初始化成员变量
    is_move_finish = false;
}

// 析构函数
EliteCSRobotSDK::~EliteCSRobotSDK() {
    // 释放资源
    s_driver->stopControl(1000);
    std::cout << "Driver Control stop" << std::endl;
    s_dashboard->stopProgram();
    std::cout << "Program stop" << std::endl;
    s_rtsi_io->disconnect();
    std::cout << "RTSI disconnected" << std::endl;
    s_rtsi_client->disconnect();
    std::cout << "RTSI client disconnected" << std::endl;
    s_dashboard->disconnect();
    std::cout << "Dashboard disconnected" << std::endl;
}

// 执行初始化操作
bool EliteCSRobotSDK::init() {
    std::cout << "Initializing..." << std::endl;
    s_driver = std::make_unique<ELITE::EliteDriver>(robot_ip, pc_ip, external_control_script);
    s_rtsi_io = std::make_unique<ELITE::RtsiIOInterface>(output_recipe, input_recipe, frequency);
    s_dashboard = std::make_unique<ELITE::DashboardClient>();
    s_rtsi_client = std::make_unique<ELITE::RtsiClientInterface>();

    if (!s_dashboard->connect(robot_ip)) {
        std::cout << "Dashboard connect false" << std::endl;
        return false;
    }
    std::cout << "Dashboard connected" << std::endl;

    if (!s_rtsi_io->connect(robot_ip)) {
        std::cout << "RTSI IO connect false" << std::endl;
        return false;
    }
    std::cout << "RTSI IO connected" << std::endl;
    
    s_rtsi_client->connect(robot_ip);
    std::cout << "RTSI client connected" << std::endl;

    return true;
}

// 启动机械臂相关服务
bool EliteCSRobotSDK::start() {
    // 加载任务文件
    if (!s_dashboard->loadTask(task_file)) {
        std::cout << "Could not load  " << task_file.c_str() << std::endl;
        return false;
    }
    std::string task = s_dashboard->getTaskPath();
    if (task!= task_file) {
        std::cout << "Not load right task" << std::endl;
        return false;
    } else {
        std::cout << "Load task:" << task << std::endl;
    }

    // 开机
    if (!s_dashboard->powerOn()) {
        std::cout << "Robot power on false" << std::endl;
        return false;
    }
    std::cout << "Robot power on" << std::endl;

    // 释放抱闸
    if (!s_dashboard->brakeRelease()) {
        std::cout << "Robot brake released false" << std::endl;
        return false;
    }
    std::cout << "Robot brake released" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 执行外部控制文件
    if (!s_dashboard->playProgram()) {
        std::cout << "Program run false" << std::endl;
        return false;
    }
    std::cout << "Program run" << std::endl;

    // 循环不断尝试连接驱动，program run后才能连接驱动
    while (!s_driver->isRobotConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "Connecting Driver..." << std::endl;
    }
    std::cout << "Driver connected" << std::endl;

    // 设置轨迹跟踪回调函数。机械臂是否移动完成
    s_driver->setTrajectoryResultCallback([&](ELITE::TrajectoryMotionResult result) {
        if (result == ELITE::TrajectoryMotionResult::SUCCESS) {
            is_move_finish = true;
        }
    });

    return true;
}

// 断开与机械臂的连接
bool EliteCSRobotSDK::disconnect() {
    s_driver->stopControl();
    std::cout << "Driver Control stop" << std::endl;
    s_dashboard->stopProgram();
    std::cout << "Program stop" << std::endl;
    s_rtsi_io->disconnect();
    std::cout << "RTSI disconnected" << std::endl;
    s_rtsi_client->disconnect();
    std::cout << "RTSI client disconnected" << std::endl;
    s_dashboard->disconnect();
    std::cout << "Dashboard disconnected" << std::endl;
    return true;
}

// 停止运动
bool EliteCSRobotSDK::stopMove() {
    if (!s_driver->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::CANCEL, 1, 200)) {
        std::cout << "STOP Move false" << std::endl;
        return false;
    }
    std::cout << "STOP Move" << std::endl;
    is_move_finish = true;

    return true;
}

// 获取当前关节位置(rad)
ELITE::vector6d_t EliteCSRobotSDK::getCurrentJoint() {
    ELITE::vector6d_t current_joint = s_rtsi_io->getActualJointPositions();
    return current_joint;
}

// 获取当前末端笛卡尔空间位姿（m）(x,y,z,rx,ry,rz)
ELITE::vector6d_t EliteCSRobotSDK::getCurrentTCPPose() {
    ELITE::vector6d_t current_TCP_Pose = s_rtsi_io->getAcutalTCPPose();
    return current_TCP_Pose;
}

// 关节移动
bool EliteCSRobotSDK::moveJoint(const ELITE::vector6d_t& joint, float time, float blend_radius) {
    is_move_finish = false;

    // 发送开始运动的指令，部位点数量为1，设置下次发送指令的超时时间为500ms（即500ms内需要给出下一条指令）。
    s_driver->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::START, 1, 500);

    // 设制轨迹点，设置当前点为起始点，设置到达时间为3s，设置半径为0.05，设置true表输入笛卡尔空间，false为关节空间。
    s_driver->writeTrajectoryPoint(joint, time, blend_radius, false);

    // 等待运动结束，等待期间持续调用`writeTrajectoryControlAction()`函数给机器人发送空指令，否则会对外部控制造成影响。
    while (!is_move_finish) {
        if (!s_driver->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
            std::cout << "Robot Move false" << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Robot Joint Move Finished" << std::endl;
    return true;
}

// 笛卡尔空间直线运动，输入末端位姿移动，单位m，rad
bool EliteCSRobotSDK::moveLine(const ELITE::vector6d_t& pose, float time, float blend_radius) {
    is_move_finish = false;

    // 发送开始运动的指令，点位数量为1，设置下次发送指令的超时时间为500ms（即500ms内需要给出下一条指令）。
    s_driver->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::START, 1, 500);

    // 设制轨迹点，设置当前点为起始点，设置到达时间为3s，设置半径为0.05，设置true表输入笛卡尔空间，false为关节空间。
    s_driver->writeTrajectoryPoint(pose, time, blend_radius, true);

    // 等待运动结束，等待期间持续调用`writeTrajectoryControlAction()`函数给机器人发送空指令，否则会对外部控制造成影响。
    while (!is_move_finish) {
        if (!s_driver->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
            std::cout << "Robot Move false" << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Robot Pose Move Finished" << std::endl;
    return true;
}

// 输入路点各关节角度(不需要起始点路点)，运行时间，进行轨迹跟踪，单位rad
bool EliteCSRobotSDK::runTrajectoryJ(const std::vector<ELITE::vector6d_t>& joint_points, const std::vector<float>& joint_times, float blend_radius) {
    if(joint_points.size() != joint_times.size()) {
        std::cout << "Joint Points's Size is NOT Equal to Joint Times's Size, please check." << std::endl;
        return false;
    }
    int point_number = joint_points.size();

    is_move_finish = false;

    s_driver->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::START, point_number, 1000);

    for (int k = 0; k < point_number; k++) {
        // 获取当前路点的关节位置数据
        ELITE::vector6d_t joint_positions = {
            joint_points[k][0],
            joint_points[k][1],
            joint_points[k][2],
            joint_points[k][3],
            joint_points[k][4],
            joint_points[k][5]
        };
        std::cout << k << "The Target point is: " << joint_positions[0] << " " << joint_positions[1]  << " " << joint_positions[2]
                                    << " " << joint_positions[3]  << " " << joint_positions[4]  << " " << joint_positions[5]<< std::endl;
        std::cout << "The Time to this point is: " << joint_times[k] << std::endl;
        s_driver->writeTrajectoryPoint(joint_positions, joint_times[k], 0.05, false);
    }

    // 等待运动结束，等待期间持续调用`writeTrajectoryControlAction()`函数给机器人发送空指令，否则会对外部控制造成影响。
    while (!is_move_finish) {
        if (!s_driver->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 1000)) {
            std::cout << "Robot Move false" << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "Robot Joints Move Finished" << std::endl;

    return true;
}

// 输入机械臂路点末端的姿态(不需要起始点路点)，运行时间，进行轨迹跟踪，单位m，rad
bool EliteCSRobotSDK::runTrajectoryC(const std::vector<ELITE::vector6d_t>& pose_points, const std::vector<float>& pose_times, float blend_radius) {
    if(pose_points.size() != pose_times.size()) {
        std::cout << "Pose Points's Size is NOT Equal to Pose Times's Size, please check." << std::endl;
        return false;
    }
    int point_number = pose_times.size();
    is_move_finish = false;

    s_driver->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::START, point_number, 500);

    for (int k = 0; k < point_number; k++) {
        // 获取当前路点的关节位置数据
        ELITE::vector6d_t pose_positions = {
            pose_points[k][0],
            pose_points[k][1],
            pose_points[k][2],
            pose_points[k][3],
            pose_points[k][4],
            pose_points[k][5]
        };

        std::cout << k << "The Target pose is: " << pose_positions[0] << " " << pose_positions[1]  << " " << pose_positions[2]
                                    << " " << pose_positions[3]  << " " << pose_positions[4]  << " " << pose_positions[5]<< std::endl;
        std::cout << "The Time to this pose is: " << pose_times[k] << std::endl;
        s_driver->writeTrajectoryPoint(pose_positions, pose_times[k], blend_radius, true);

        // 等待运动结束，等待期间持续调用`writeTrajectoryControlAction()`函数给机器人发送空指令，否则会对外部控制造成影响。
        while (!is_move_finish) {
            if (!s_driver->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
                std::cout << "Robot Move false" << std::endl;
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "Robot Poses Move Finished" << std::endl;
    }

    return true;
}

// 控制各关节速度 rad/s
bool EliteCSRobotSDK::jointSpeed(const ELITE::vector6d_t& speed, int timeout_ms) {
    if(!s_driver->writeSpeedj(speed,timeout_ms)) {
        std::cout << "joint Speed control false" << std::endl;
        return false;
    }
    return true;
}

bool EliteCSRobotSDK::lineSpeed(const ELITE::vector6d_t& speed, int timeout_ms) {
    if(!s_driver->writeSpeedl(speed,timeout_ms)) {
        std::cout << "line Speed control false" << std::endl;
        return false;
    }
    return true;
}

void EliteCSRobotSDK::test() {std::cout<<"successful"<<std::endl;}