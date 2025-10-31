#include <Elite/DashboardClient.hpp>
#include <Elite/DataType.hpp>
#include <Elite/EliteDriver.hpp>
#include <Elite/Log.hpp>
#include <Elite/RtsiIOInterface.hpp>

// 控制cs机械臂的类，需要指定机械臂IP和电脑IP，外部控制脚本，rtsi文件，频率，有默认值
class EliteCSRobotSDK {
private:
    // 机械臂相关对象
    std::unique_ptr<ELITE::EliteDriver> s_driver;
    std::unique_ptr<ELITE::RtsiIOInterface> s_rtsi_io;
    std::unique_ptr<ELITE::RtsiClientInterface> s_rtsi_client;
    std::unique_ptr<ELITE::DashboardClient> s_dashboard;

    // 机械臂状态信息
    std::string robot_ip;
    std::string pc_ip;
    std::string external_control_script;
    std::string output_recipe;
    std::string input_recipe;
    double frequency;
    std::string task_file;  //用于加载的任务（外部控制）
    bool is_move_finish;
    

public:
    // 构造函数，指定机器人IP和电脑IP，外部控制文件，rtsi文件，频率，加载任务名，部分有默认值
    EliteCSRobotSDK(const std::string& robot_ip = "192.168.1.200", const std::string& pc_ip = "192.168.1.150",
    const std::string& external_control_script = "external_control.script",
    const std::string& output_recipe = "output_recipe.txt",
    const std::string& input_recipe = "input_recipe.txt",
    const std::string& task_file = "mjktest.task",
    double frequency = 250);

    // 析构函数
    ~EliteCSRobotSDK();

    // 执行连接驱动和接口等初始化操作
    bool init();

    // 启动机械臂相关服务:释放抱闸，连接外部控制驱动等
    bool start();

    // 断开与机械臂的连接
    bool disconnect();

    // 获取当前关节位置，单位rad
    ELITE::vector6d_t getCurrentJoint();

    // 获取当前末端笛卡尔空间位姿，单位m
    ELITE::vector6d_t getCurrentTCPPose();

    // 关节移动，暂用轨迹跟踪控制实现，单位rad
    bool moveJoint(const ELITE::vector6d_t& joint, float time = 3, float blend_radius = 0.05);

    // 笛卡尔空间直线运动，输入末端位姿移动，单位m，rad
    bool moveLine(const ELITE::vector6d_t& pose, float time = 3, float blend_radius = 0.05);

    // 停止运动
    bool stopMove();

    // 输入路点各关节角度(不需要起始点路点)，运行时间，进行轨迹跟踪，单位rad
    bool runTrajectoryJ(const std::vector<ELITE::vector6d_t>& joint_points, const std::vector<float>& joint_times, float blend_radius = 0.05);

    // 输入机械臂路点末端的姿态(不需要起始点路点)，运行时间，进行轨迹跟踪，单位m，rad
    bool runTrajectoryC(const std::vector<ELITE::vector6d_t>& pose_points, const std::vector<float>& pose_times, float blend_radius = 0.05);

    // 控制各关节速度 rad/s
    bool jointSpeed(const ELITE::vector6d_t& speed, int timeout_ms = 0);

    // 控制末端点速度，{x,y,z,rx,ry,rz} m/s,rad/s
    bool lineSpeed(const ELITE::vector6d_t& speed, int timeout_ms = 0);

    void test();

};