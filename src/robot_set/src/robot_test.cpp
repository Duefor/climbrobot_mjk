/**
 * 力控调试代码：机械臂平面直线运动 + Z方向恒力控制(5N)
 *
 * 流程：
 *   1. 从当前点移动到设定的起始点（平面上，末端垂直向下）
 *   2. 开启力控（仅Z方向柔顺，维持5N恒力）
 *   3. 沿直线移动到终点（平面运动，力控保持）
 *   4. 关闭力控
 *   5. 返回初始点
 */

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <atomic>
#include <ctime>

#include <termios.h>
#include <unistd.h>

#include <robot_sdk_wrapper/robot_sdk.h>

// 等待键盘 Enter 按下，显示提示信息
void waitForEnter(const std::string& next_step_desc) {
    std::cout << "\n>>> 按 Enter 键执行: " << next_step_desc << " <<<" << std::flush;

    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    while (true) {
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0 && (c == '\r' || c == '\n')) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
    std::cout << std::endl;
}

// 生成带时间戳的 CSV 文件名
std::string makeLogFilename() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm* local = std::localtime(&t);
    std::ostringstream oss;
    oss << "force_log_"
        << std::setfill('0')
        << std::setw(4) << local->tm_year + 1900
        << std::setw(2) << local->tm_mon + 1
        << std::setw(2) << local->tm_mday << "_"
        << std::setw(2) << local->tm_hour
        << std::setw(2) << local->tm_min
        << std::setw(2) << local->tm_sec
        << ".csv";
    return oss.str();
}

// 力传感器数据日志记录器
// 在独立线程中运行，以 ~250Hz 采集力/位姿数据写入 CSV
class ForceLogger {
public:
    ForceLogger(EliteCSRobotSDK* robot, const std::string& filename)
        : robot_(robot), filename_(filename), running_(false) {}

    void start() {
        running_ = true;
        start_time_ = std::chrono::steady_clock::now();
        worker_ = std::thread(&ForceLogger::run, this);
    }

    void stop() {
        running_ = false;
        if (worker_.joinable()) {
            worker_.join();
        }
    }

private:
    void run() {
        std::ofstream file(filename_);
        if (!file.is_open()) {
            std::cerr << "[ERROR] 无法创建日志文件: " << filename_ << std::endl;
            return;
        }

        // CSV 表头
        file << "timestamp_s,Fx_N,Fy_N,Fz_N,Mx_Nm,My_Nm,Mz_Nm,"
             << "TCP_x_m,TCP_y_m,TCP_z_m,TCP_rx_rad,TCP_ry_rad,TCP_rz_rad\n";
        file << std::fixed << std::setprecision(6);

        std::cout << "[INFO] 力数据日志已启动 → " << filename_ << std::endl;

        constexpr auto kSamplePeriod = std::chrono::microseconds(4000);  // ~250Hz
        auto next_wake = std::chrono::steady_clock::now();

        while (running_) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start_time_).count();

            ELITE::vector6d_t force = robot_->getTCPforce();
            ELITE::vector6d_t pose  = robot_->getCurrentTCPPose();

            file << elapsed << ","
                 << force[0] << "," << force[1] << "," << force[2] << ","
                 << force[3] << "," << force[4] << "," << force[5] << ","
                 << pose[0]  << "," << pose[1]  << "," << pose[2]  << ","
                 << pose[3]  << "," << pose[4]  << "," << pose[5]  << "\n";

            next_wake += kSamplePeriod;
            std::this_thread::sleep_until(next_wake);
        }

        file.close();
        std::cout << "[INFO] 力数据日志已保存 → " << filename_ << std::endl;
    }

    EliteCSRobotSDK* robot_;
    std::string filename_;
    std::atomic<bool> running_;
    std::thread worker_;
    std::chrono::steady_clock::time_point start_time_;
};

const std::string DEFAULT_ROBOT_IP = "192.168.1.199";
const std::string DEFAULT_PC_IP = "192.168.1.150";
const std::string external_control_file_address =
    "/home/barry/workspace/climbrobot_mjk/src/robot_sdk_wrapper/resource/external_control.script";
const std::string output_recipe_file_address =
    "/home/barry/workspace/climbrobot_mjk/src/robot_sdk_wrapper/resource/output_recipe.txt";
const std::string input_recipe_file_address =
    "/home/barry/workspace/climbrobot_mjk/src/robot_sdk_wrapper/resource/input_recipe.txt";
const std::string task_file_address = "test12.task";

// ---- 可调参数 ----
// 平面运动高度 (m)
constexpr double kWorkPlaneZ = 0.098;

// 起始点: XY 平面位置 (m)，姿态保持末端垂直向下 (rx=π, ry=0, rz=0)
constexpr double kStartX = 0.295;
constexpr double kStartY = -0.456;

// 终点: 沿 X 方向移动一段距离
constexpr double kEndX = 0.58;
constexpr double kEndY = -0.16;

// 运动时间 (s)
constexpr double kMoveTime = 15.0;

// 力控目标力 (N)，仅 Z 方向
constexpr double kForceTargetZ = 5.0;

// 力控 Z 方向最大速度 (m/s)
constexpr double kForceLimitZ = 0.04;

// 末端垂直向下的姿态: rx = π (绕X轴180°), ry = 0, rz = 0
constexpr double kToolRx = M_PI;
constexpr double kToolRy = 0.0;
constexpr double kToolRz = 0.0;

int main(int argc, char** argv)
{
    std::cout << "========== 力控调试程序 ==========" << std::endl;

    // ---- 1. 初始化机械臂 ----
    EliteCSRobotSDK cs66robot(
        DEFAULT_ROBOT_IP, DEFAULT_PC_IP, false,
        external_control_file_address,
        output_recipe_file_address,
        input_recipe_file_address,
        task_file_address, 250);

    if (!cs66robot.init()) {
        std::cout << "[ERROR] Robot init failed" << std::endl;
        return 1;
    }
    std::cout << "[INFO] Robot init successful" << std::endl;

    if (!cs66robot.start()) {
        std::cout << "[ERROR] Robot start failed" << std::endl;
        return 1;
    }
    std::cout << "[INFO] Robot start successful" << std::endl;

    // ---- 2. 记录当前点作为最终返回点 ----
    ELITE::vector6d_t initial_pose = cs66robot.getCurrentTCPPose();
    std::cout << "[INFO] 当前初始点 TCP: "
              << initial_pose[0] << ", " << initial_pose[1] << ", " << initial_pose[2] << ", "
              << initial_pose[3] << ", " << initial_pose[4] << ", " << initial_pose[5] << std::endl;

    waitForEnter("移动到起始点 (moveLine → start_pose)");

    // ---- 3. 移动到设定的起始点（平面运动，末端垂直） ----
    ELITE::vector6d_t start_pose = {kStartX, kStartY, kWorkPlaneZ, kToolRx, kToolRy, kToolRz};
    std::cout << "[INFO] 移动到起始点: "
              << start_pose[0] << ", " << start_pose[1] << ", " << start_pose[2] << ", "
              << start_pose[3] << ", " << start_pose[4] << ", " << start_pose[5] << std::endl;

    if (!cs66robot.moveLine(start_pose, kMoveTime)) {
        std::cout << "[ERROR] 移动到起始点失败" << std::endl;
        cs66robot.disconnect();
        return 1;
    }
    std::cout << "[INFO] 到达起始点" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    waitForEnter("开启力控 (Z方向柔顺, 5N恒力)");

    // ---- 4. 开启力控：仅 Z 方向柔顺，维持 5N 恒力 ----
    // reference_frame: 以当前 TCP 位姿为力参考坐标系
    // selection_vector: {0,0,1,0,0,0} 表示仅 Z 轴柔顺
    // wrench: {0,0,5,0,0,0} 目标力 Z=5N
    // limits: {0,0,0.02,0,0,0} Z 方向最大速度 0.02 m/s
    ELITE::vector6d_t reference_frame = cs66robot.getCurrentTCPPose();
    ELITE::vector6int32_t selection_vector = {0, 0, 1, 0, 0, 0};
    ELITE::vector6d_t wrench = {0.0, 0.0, kForceTargetZ, 0.0, 0.0, 0.0};
    ELITE::vector6d_t limits = {0.0, 0.0, kForceLimitZ, 0.0, 0.0, 0.0};

    std::cout << "[INFO] 开启力控: Z方向柔顺, 目标力=" << kForceTargetZ
              << "N, 最大速度=" << kForceLimitZ << "m/s" << std::endl;

    if (!cs66robot.startForceControl(reference_frame, selection_vector, wrench, limits)) {
        std::cout << "[ERROR] 开启力控失败" << std::endl;
        cs66robot.disconnect();
        return 1;
    }
    std::cout << "[INFO] 力控已开启" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    waitForEnter("力控模式下直线移动到终点 (moveLine → end_pose)");

    // ---- 5. 力控模式下沿直线移动到终点（同时记录力数据） ----
    ELITE::vector6d_t end_pose = {kEndX, kEndY, kWorkPlaneZ, kToolRx, kToolRy, kToolRz};
    std::cout << "[INFO] 力控模式下直线移动到终点: "
              << end_pose[0] << ", " << end_pose[1] << ", " << end_pose[2] << ", "
              << end_pose[3] << ", " << end_pose[4] << ", " << end_pose[5] << std::endl;

    // 启动力数据日志记录
    ForceLogger logger(&cs66robot, makeLogFilename());
    logger.start();

    if (!cs66robot.moveLine(end_pose, kMoveTime)) {
        logger.stop();
        std::cout << "[ERROR] 力控模式下移动到终点失败" << std::endl;
        cs66robot.endForceControl();
        cs66robot.disconnect();
        return 1;
    }

    // 停止日志记录
    logger.stop();
    std::cout << "[INFO] 到达终点" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    waitForEnter("关闭力控");

    // ---- 6. 关闭力控 ----
    if (!cs66robot.endForceControl()) {
        std::cout << "[ERROR] 关闭力控失败" << std::endl;
        cs66robot.disconnect();
        return 1;
    }
    std::cout << "[INFO] 力控已关闭" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    waitForEnter("返回初始点 (moveLine → initial_pose)");

    // ---- 7. 返回初始点 ----
    std::cout << "[INFO] 返回初始点: "
              << initial_pose[0] << ", " << initial_pose[1] << ", " << initial_pose[2] << ", "
              << initial_pose[3] << ", " << initial_pose[4] << ", " << initial_pose[5] << std::endl;

    if (!cs66robot.moveLine(initial_pose, kMoveTime)) {
        std::cout << "[ERROR] 返回初始点失败" << std::endl;
        cs66robot.disconnect();
        return 1;
    }
    std::cout << "[INFO] 已返回初始点" << std::endl;

    // ---- 8. 断开连接 ----
    cs66robot.disconnect();
    std::cout << "========== 力控调试程序完成 ==========" << std::endl;
    return 0;
}
