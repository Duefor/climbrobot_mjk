#ifndef CS66_ROBOT_CONTROLLER_H
#define CS66_ROBOT_CONTROLLER_H

#include <Elite/DashboardClient.hpp>
#include <Elite/DataType.hpp>
#include <Elite/EliteDriver.hpp>
#include <Elite/Log.hpp>
#include <Elite/RtsiIOInterface.hpp>
#include <Elite/RtUtils.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <cmath>
#include <mutex>

#if defined(__linux) || defined(linux) || defined(__linux__)
#include <sys/mman.h>
#include <pthread.h>
#endif

using namespace ELITE;

namespace moveit {
namespace planning_interface {
class MoveGroupInterface;
}
}

struct AdmittanceForceControlParams {
    // 期望的 TCP-Z 力传感器读数，单位 N。末端朝下接触 8N 时通常约为 -8N。
    double target_force_z = -8.0;
    // 力误差方向系数。默认力不足时沿名义 TCP 自身 +Z 方向补偿；方向反了设为 -1。
    double force_sign = 1.0;
    double admittance_mass = 1.0;   // 质量参数，单位 kg
    double admittance_damping = 30.0;   // 阻尼参数，单位 N/(m/s)
    double admittance_stiffness = 80.0;  // 刚度参数，单位 N/m
    double force_filter_alpha = 0.15;   // 低通滤波系数，范围 [0,1]，越大滤波越慢
    double control_period = 0.01;   // 控制周期，单位 s，建议 >= 0.002
    double nominal_tcp_speed = 0.02;    // 名义 TCP 速度，单位 m/s，用于计算轨迹总时长
    double max_z_offset = 0.05;  // TCP-Z 最大偏移量，单位 m
    double max_z_velocity = 0.001;   // TCP-Z 最大速度，单位 m/s
    double max_joint_step = 0.03;   // 最大关节步长，单位 rad，防止关节跳跃
    double force_abort_threshold = 25.0;    // 当滤波后的 TCP-Z 力超过此阈值时，立即中止轨迹执行，单位 N
    double state_timeout = 0.2; // 读取缓存状态的超时时间，单位 s
    int max_consecutive_ik_failures = 3;    // 连续 IK 失败次数超过此值时，立即中止轨迹执行
};

/**
 * @brief CS66 Robot Controller Class
 * 
 * This class provides a ROS interface for controlling CS66 robots using the Elite SDK.
 * It handles robot initialization, status publishing, and motion control through ROS topics.
 */
class CS66RobotController {
private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 发布器
    ros::Publisher joint_state_pub_;
    ros::Publisher tcp_pose_pub_;
    ros::Publisher tcp_force_pub_;


    // 订阅器
    ros::Subscriber joint_command_sub_;
    ros::Subscriber tcp_command_sub_;

    // 定时器
    ros::Timer status_timer_;
    ros::Timer keepalive_timer_;  // 保持连接定时器，定期发送 IDLE 指令防止机器人端脚本超时退出
    
    // 队列模式配置参数
    int servoj_queue_pre_recv_size_;
    float servoj_queue_pre_recv_timeout_ms_;
    bool servo_mode_enabled_;
    
    // Elite SDK组件
    std::unique_ptr<EliteDriver> driver_;
    std::unique_ptr<DashboardClient> dashboard_;
    std::unique_ptr<RtsiIOInterface> rtsi_client_;
    
    // 配置参数
    EliteDriverConfig config_;
    std::string robot_ip_;
    std::string local_ip_;
    bool headless_mode_;
    double control_frequency_;
    double status_frequency_;
    // 轨迹插值相关参数
    double max_velocity;
    
    // 状态变量
    bool is_initialized_;
    bool is_connected_;
    bool emergency_stop_;
    bool is_move_finish_;

    struct CachedRobotState {
        ELITE::vector6d_t actual_joints{};
        ELITE::vector6d_t actual_tcp_pose_axis_angle{};
        ELITE::vector6d_t actual_tcp_force{};
        ros::Time stamp;
        bool valid = false;
    };
    mutable std::mutex cached_state_mutex_;
    CachedRobotState cached_state_;
    
    // 话题名称
    std::string joint_state_topic_;
    std::string tcp_pose_topic_;
    std::string joint_command_topic_;
    std::string tcp_command_topic_;
    std::string tcp_force_topic_;




    // 私有方法
    void loadParameters();
    void initializeROSInterface();
    void initializeEliteSDK();
    void updateCachedRobotState();
    bool copyCachedRobotState(CachedRobotState& state, double max_age_sec) const;
    
    // 状态发布
    void publishJointStates();
    void publishTCPPose();
    void publishTCPForce();
    void statusTimerCallback(const ros::TimerEvent&);
    void keepaliveTimerCallback(const ros::TimerEvent&);  
    
    // 话题回调
    // void jointCommandCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    // void tcpCommandCallback(const geometry_msgs::Pose::ConstPtr& msg);
public:
    /**
     * @brief Constructor
     * @param nh ROS node handle
     * @param private_nh ROS private node handle
     */
    CS66RobotController(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    
    /**
     * @brief Destructor
     */
    ~CS66RobotController();

    bool RunJointTrajectory(const std::vector<ELITE::vector6d_t>& trajectory, 
        const std::vector<double>& joint_times, int point_number, float blend_radius);
    bool RunPoseTrajectory(const std::vector<ELITE::vector6d_t>& trajectory, 
        const std::vector<double>& pose_times, int point_number, float blend_radius);
    
    bool RunServojTrajectory(const std::vector<ELITE::vector6d_t>& trajectory, 
        const std::vector<double>& times);

    bool RunJointServoAdmittanceTrajectory(
        const std::vector<geometry_msgs::Pose>& nominal_poses,
        moveit::planning_interface::MoveGroupInterface& move_group,
        const AdmittanceForceControlParams& params = AdmittanceForceControlParams());

    bool WriteServojJointOnce(const ELITE::vector6d_t& joints, int timeout_ms = 100);

    bool WriteIdleOnce(int timeout_ms = 0);

    bool SendScriptToRobot(const std::string& script);

    // Force mode control: simplified API - only provide desired Z force (N)
    bool startForceMode(const ELITE::vector6d_t& ref_frame, const vector6int32_t&, const vector6d_t&, const vector6d_t&);

    bool endForceMode();

    std::vector<ELITE::vector6d_t> GenerateDenseTrajectory(
        const std::vector<ELITE::vector6d_t>& raw_points,
        double max_velocity,
        double servo_period);

    // 基于时间的密集轨迹生成（按每点时间戳插值）
    std::vector<ELITE::vector6d_t> GenerateDenseTrajectoryTimed(
        const std::vector<ELITE::vector6d_t>& raw_points,
        const std::vector<double>& point_times,
        double servo_period);

    
    
    bool motionFinished() const { return is_move_finish_; }
    void resetMotionFinish() { is_move_finish_ = false; }
    void stopRobot();

    // Note: CatmullRom and SlerpRotation are implemented as free functions
    // in the .cpp file and do not need to be declared here.
    
};

#endif // CS66_ROBOT_CONTROLLER_H
