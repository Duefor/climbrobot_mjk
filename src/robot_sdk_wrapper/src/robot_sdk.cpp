#include "robot_sdk_wrapper/robot_sdk.h"
#include <algorithm>
#include <iostream>
#include <cmath>
#include <chrono>

namespace {
// 将目标关节角限制为相对参考点的单周期安全步长。
// 避免单次跳变超过 max_step，以防触发底层驱动的速度保护。
ELITE::vector6d_t limitJointStep(const ELITE::vector6d_t& reference,
                                 const ELITE::vector6d_t& target,
                                 double max_step) {
    ELITE::vector6d_t limited = target;
    for (int i = 0; i < 6; ++i) {
        double delta = target[i] - reference[i];
        // 用 std::max/std::min 将 delta 限制在 [-max_step, max_step] 范围内
        double clamped_delta = std::max(-max_step, std::min(delta, max_step));
        limited[i] = reference[i] + clamped_delta;
    }
    return limited;
}
}  // namespace

// 构造函数
// EliteCSRobotSDK::EliteCSRobotSDK(const std::string& robot_ip, const std::string& pc_ip, const std::string& external_control_script, double mode,
//                          const std::string& output_recipe, const std::string& input_recipe, const std::string& task_file ,double frequency) :
//         robot_ip(robot_ip), pc_ip(pc_ip), external_control_script(external_control_script),
//         output_recipe(output_recipe), input_recipe(input_recipe), task_file(task_file), frequency(frequency) {
//     // 初始化成员变量
//     is_move_finish = false;
//     DriverConfig.robot_ip = robot_ip;
//     DriverConfig.local_ip = pc_ip;
//     DriverConfig.script_file_path = external_control_script;
//     DriverConfig.headless_mode = false;
//     // 剩余参数用默认
//     DriverConfig.script_sender_port = 50002;
//     DriverConfig.reverse_port = 50001;
//     DriverConfig.trajectory_port = 50003;
//     DriverConfig.script_command_port = 50004;
//     DriverConfig.servoj_time = 0.008;
//     DriverConfig.servoj_lookahead_time = 0.1;
//     DriverConfig.servoj_gain = 300;
//     DriverConfig.stopj_acc = 8;
//     DriverConfig.servoj_queue_pre_recv_size = 10;
//     DriverConfig.servoj_queue_pre_recv_timeout = -1;
// }

EliteCSRobotSDK::EliteCSRobotSDK(const std::string& robot_ip, const std::string& pc_ip, bool mode,
    const std::string& external_control_script,
    const std::string& output_recipe,
    const std::string& input_recipe,
    const std::string& task_file,
    double frequency):
    robot_ip(robot_ip),
    pc_ip(pc_ip), 
    external_control_script(external_control_script),
    output_recipe(output_recipe), 
    input_recipe(input_recipe), 
    task_file(task_file), 
    frequency(frequency) {
    // 初始化成员变量
    is_move_finish = false;
    DriverConfig.robot_ip = robot_ip;
    DriverConfig.local_ip = pc_ip;
    DriverConfig.script_file_path = external_control_script;
    DriverConfig.headless_mode = mode;
    // 剩余参数用默认
    DriverConfig.script_sender_port = 50002;
    DriverConfig.reverse_port = 50001;
    DriverConfig.trajectory_port = 50003;
    DriverConfig.script_command_port = 50004;
    DriverConfig.servoj_time = 0.008;
    DriverConfig.servoj_lookahead_time = 0.1;
    DriverConfig.servoj_gain = 300;
    DriverConfig.stopj_acc = 8;
    DriverConfig.servoj_queue_pre_recv_size = 10;
    DriverConfig.servoj_queue_pre_recv_timeout = -1;

    // 关节伺服默认采用较保守的限幅，避免直接把大跳变发到底层驱动。
    servoj_max_joint_velocity_ = 2.0;
    servoj_servo_period_ = 0.008;

}

// 析构函数
EliteCSRobotSDK::~EliteCSRobotSDK() {
    // 释放资源
    s_driver->stopControl(1000);
    // 老版sdk无这个参数
    // s_driver->stopControl();
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
    // s_driver = std::make_unique<ELITE::EliteDriver>(robot_ip, pc_ip, external_control_script);
    s_driver = std::make_unique<ELITE::EliteDriver>(DriverConfig);
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

bool EliteCSRobotSDK::init_read_data() {
    std::cout << "Initializing..." << std::endl;
    // s_driver = std::make_unique<ELITE::EliteDriver>(robot_ip, pc_ip, external_control_script);
    // s_driver = std::make_unique<ELITE::EliteDriver>(DriverConfig);
    s_rtsi_io = std::make_unique<ELITE::RtsiIOInterface>(output_recipe, input_recipe, frequency);
    // s_dashboard = std::make_unique<ELITE::DashboardClient>();
    s_rtsi_client = std::make_unique<ELITE::RtsiClientInterface>();

    // if (!s_dashboard->connect(robot_ip)) {
    //     std::cout << "Dashboard connect false" << std::endl;
    //     return false;
    // }
    // std::cout << "Dashboard connected" << std::endl;

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
    if(DriverConfig.headless_mode)
    {
        if(!EliteCSRobotSDK::startMode1())
        {
            std::cout << "Mode 1 (external) start error" << std::endl;
        }
    }
    else
    {
        if(!EliteCSRobotSDK::startMode2())
        {
            std::cout << "Mode 2 (task) start error" << std::endl;
        }
    }

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

    // 新一轮运行开始前，清空上一轮的关节伺服参考值。
    resetServojTrackingState();

    return true;
}

bool EliteCSRobotSDK::startMode1() {
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

    // 发送外部控制文件
    if (!s_driver->isRobotConnected()) {
        if (!s_driver->sendExternalControlScript()) {
            std::cout << "Fail to send external control script" << std::endl;
            return false;
        }
    }
    return true;
}

bool EliteCSRobotSDK::startMode2() {
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

    if (!s_dashboard->playProgram()) {
        std::cout << "Robot playProgram false" << std::endl;
        return false;
    }

    std::cout << "Robot playProgram" << std::endl;

    return true;
}

// 断开与机械臂的连接
bool EliteCSRobotSDK::disconnect() {
    resetServojTrackingState();
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
    resetServojTrackingState();

    return true;
}

// 获取当前关节位置(rad)
ELITE::vector6d_t EliteCSRobotSDK::getCurrentJoint() {
    ELITE::vector6d_t current_joint = s_rtsi_io->getActualJointPositions();
    return current_joint;
}

// 获取当前关节速度(rad/s)
ELITE::vector6d_t EliteCSRobotSDK::getCurrentJointVelocity() {
    ELITE::vector6d_t current_joint_velocity = s_rtsi_io->getActualJointVelocity();
    return current_joint_velocity;
}

// 获取当前末端笛卡尔空间位姿（m）(x,y,z,rx,ry,rz)
ELITE::vector6d_t EliteCSRobotSDK::getCurrentTCPPose() {
    ELITE::vector6d_t current_TCP_Pose = s_rtsi_io->getAcutalTCPPose();
    return current_TCP_Pose;
}

// 获取当前末端笛卡尔空间速度（m/s）(x,y,z,rx,ry,rz)
ELITE::vector6d_t EliteCSRobotSDK::getCurrentTCPVelocity() {
    ELITE::vector6d_t current_TCP_velocity = s_rtsi_io->getAcutalTCPVelocity();
    return current_TCP_velocity;
}

// 关节移动
bool EliteCSRobotSDK::moveJoint(const ELITE::vector6d_t& joint, float time, float blend_radius) {
    is_move_finish = false;

    // 发送开始运动的指令，部位点数量为1，设置下次发送指令的超时时间为500ms（即500ms内需要给出下一条指令）。
    s_driver->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::START, 1, 500);

    // 设制轨迹点，设置当前点为起始点，设置到达时间为10s，设置半径为0.05，设置true表输入笛卡尔空间，false为关节空间。
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

bool EliteCSRobotSDK::moveJoint_servo(const ELITE::vector6d_t& joint, float arrive_time, float servo_period){
    ELITE::vector6d_t q_start = s_rtsi_io->getActualJointPositions();

    int N = std::ceil(arrive_time / servo_period);
    if (N <= 0) return false;

    auto next = std::chrono::steady_clock::now();
    auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(servo_period));

    for (int i = 1; i <= N; ++i) {
        double t = i * servo_period;
        double tau = t / arrive_time;
        if (tau > 1.0) tau = 1.0;

        // 三次时间标定（推荐）
        double s = 3 * tau * tau - 2 * tau * tau * tau;

        ELITE::vector6d_t q_cmd;
        for (int j = 0; j < 6; ++j) {
            q_cmd[j] = q_start[j] + s * (joint[j] - q_start[j]);
        }

        if (!s_driver->writeServoj(q_cmd, 100)) {
            return false;
        }

        next += period;
        std::this_thread::sleep_until(next);
    }

    for (int i = 1; i <= 15; ++i) {
        if (!s_driver->writeServoj(joint, 100)) {
            return false;
        }

        next += period;
        std::this_thread::sleep_until(next);
    }

    return true;
}

// 笛卡尔空间直线运动，输入末端位姿移动，单位m，rad
bool EliteCSRobotSDK::moveLine(const ELITE::vector6d_t& pose, float time, float blend_radius) {
    is_move_finish = false;

    // 发送开始运动的指令，点位数量为1，设置下次发送指令的超时时间为500ms（即500ms内需要给出下一条指令）。
    s_driver->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::START, 1, 500);

    // 设制轨迹点，设置当前点为起始点，设置到达时间为10s，设置半径为0.05，设置true表输入笛卡尔空间，false为关节空间。
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

bool EliteCSRobotSDK::test(const ELITE::vector6d_t& ex_force)
{
    return s_rtsi_io->setExternalForceTorque(ex_force);
}

bool EliteCSRobotSDK::writeservoj(const ELITE::vector6d_t& pos, int timeout_ms, bool cartesian, bool queue_mode)
{
    if (!s_driver) {
        return false;
    }

    // 笛卡尔模式保留给底层控制器自身的跟踪能力，SDK 不再做 IK 转换。
    if (cartesian) {
        return s_driver->writeServoj(pos, timeout_ms, true, queue_mode);
    }

    if (!s_rtsi_io) {
        return s_driver->writeServoj(pos, timeout_ms, false, queue_mode);
    }

    // 关节模式下，SDK 只向目标逼近一个安全步长，避免单次跳变触发速度保护。
    ELITE::vector6d_t current_joint = s_rtsi_io->getActualJointPositions();
    const double now_sec = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();

    ELITE::vector6d_t limited_cmd = pos;
    {
        std::lock_guard<std::mutex> lk(servoj_mtx_);
        if (!servoj_joint_target_valid_) {
            // 第一次进入关节伺服时，先把参考点对齐到当前实际关节。
            servoj_last_joint_cmd_ = current_joint;
            servoj_joint_target_valid_ = true;
        }

        double elapsed_sec = servoj_servo_period_;
        if (servoj_joint_time_valid_) {
            elapsed_sec = std::max(0.0, now_sec - servoj_last_joint_stamp_sec_);
        }
        // 用“更短的实际间隔”和“驱动的伺服周期”共同约束单次步长，避免 250Hz 循环把目标抖成超速命令。
        const double effective_period = std::min(elapsed_sec, servoj_servo_period_);
        const double max_step = std::max(0.0, servoj_max_joint_velocity_ * effective_period);

        limited_cmd = limitJointStep(servoj_last_joint_cmd_, pos, max_step);
        servoj_last_joint_cmd_ = limited_cmd;
        servoj_last_joint_stamp_sec_ = now_sec;
        servoj_joint_time_valid_ = true;
    }

    return s_driver->writeServoj(limited_cmd, timeout_ms, false, queue_mode);
}

void EliteCSRobotSDK::resetServojTrackingState() {
    std::lock_guard<std::mutex> lk(servoj_mtx_);
    servoj_joint_target_valid_ = false;
    servoj_joint_time_valid_ = false;
    for (int i = 0; i < 6; ++i) {
        servoj_last_joint_cmd_[i] = 0.0;
    }
    servoj_last_joint_stamp_sec_ = 0.0;
}

bool EliteCSRobotSDK::ExecuteJointTrajectory(const std::vector<TrajectoryPoint>& traj, double control_freq, int IA)
{
    stop_requested_ = false;
    if (traj.size() < 2)
        return false;

    double servo_period = 1.0 / control_freq;

    auto next = std::chrono::steady_clock::now();
    auto start = std::chrono::steady_clock::now();
    auto period = std::chrono::duration<double>(servo_period);

    double total_time = traj.back().time_from_start;

    while (true)
    {
        if (stop_requested_)
        {
            s_driver->writeIdle(0);
            return false;
        }
        // double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start + period).count();
        double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start ).count();

        if (t > total_time)
            break;

        ELITE::vector6d_t cmd;

        if (IA == 0)
        {
            if (!CubicInterpolation(traj, t, cmd))
                return false;
        }
        else if (IA == 1)
        {
            if (!QuinticInterpolation(traj, t, cmd))
                return false;
        }
        else
        {
            std::cout << "未定义插值算法" << std::endl;
            return false;
        }

        if (!s_driver->writeServoj(cmd, 100, false, false))
            return false;

        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next);
    }
    s_driver->writeIdle(0);

    return true;
}

bool EliteCSRobotSDK::CubicInterpolation(const std::vector<TrajectoryPoint>& traj, double t, ELITE::vector6d_t& q_out)
{
    if (t >= traj.back().time_from_start)
    {
        q_out = traj.back().positions;
        return true;
    }

    for (size_t i = 1; i < traj.size(); ++i)
    {
        if (t <= traj[i].time_from_start)
        {
            const auto& p0 = traj[i-1];
            const auto& p1 = traj[i];

            double t0 = p0.time_from_start;
            double t1 = p1.time_from_start;
            double dt = t1 - t0;
            double tau = (t - t0) / dt;

            double h00 = 2*tau*tau*tau - 3*tau*tau + 1;
            double h10 = tau*tau*tau - 2*tau*tau + tau;
            double h01 = -2*tau*tau*tau + 3*tau*tau;
            double h11 = tau*tau*tau - tau*tau;


            for (size_t j = 0; j < 6; ++j)
            {
                double q0 = p0.positions[j];
                double q1 = p1.positions[j];
                double v0 = p0.velocities.empty() ? 0 : p0.velocities[j];
                double v1 = p1.velocities.empty() ? 0 : p1.velocities[j];

                q_out[j] =
                    h00*q0 +
                    h10*dt*v0 +
                    h01*q1 +
                    h11*dt*v1;
            }
            return true;
        }
    }
    return false;
}

bool EliteCSRobotSDK::QuinticInterpolation(const std::vector<TrajectoryPoint>& traj, double t, ELITE::vector6d_t& q_out)
{
    if (t >= traj.back().time_from_start)
    {
        q_out = traj.back().positions;
        return true;
    }

    for (size_t i = 1; i < traj.size(); ++i)
    {
        if (t <= traj[i].time_from_start)
        {
            const auto& p0 = traj[i-1];
            const auto& p1 = traj[i];

            double t0 = p0.time_from_start;
            double t1 = p1.time_from_start;
            double dt = t1 - t0;
            double tau = (t - t0) / dt;

            double tau2 = tau*tau;
            double tau3 = tau2*tau;
            double tau4 = tau3*tau;
            double tau5 = tau4*tau;

            double h0 = 1 - 10*tau3 + 15*tau4 - 6*tau5;
            double h1 = tau - 6*tau3 + 8*tau4 - 3*tau5;
            double h2 = 0.5*tau2 - 1.5*tau3 + 1.5*tau4 - 0.5*tau5;
            double h3 = 10*tau3 - 15*tau4 + 6*tau5;
            double h4 = -4*tau3 + 7*tau4 - 3*tau5;
            double h5 = 0.5*tau3 - tau4 + 0.5*tau5;

            for (size_t j = 0; j < 6; ++j)
            {
                double q0 = p0.positions[j];
                double q1 = p1.positions[j];

                double v0 = p0.velocities.empty() ? 0 : p0.velocities[j];
                double v1 = p1.velocities.empty() ? 0 : p1.velocities[j];

                double a0 = p0.accelerations.empty() ? 0 : p0.accelerations[j];
                double a1 = p1.accelerations.empty() ? 0 : p1.accelerations[j];

                q_out[j] =
                    h0*q0 +
                    h1*dt*v0 +
                    h2*dt*dt*a0 +
                    h3*q1 +
                    h4*dt*v1 +
                    h5*dt*dt*a1;
            }

            return true;
        }
    }

    return false;
}

void EliteCSRobotSDK::ExecuteJointTrajectoryStop()
{
    stop_requested_ = true;
}

ELITE::vector6d_t EliteCSRobotSDK::getTCPforce()
{
    return s_rtsi_io->getAcutalTCPForce();
}