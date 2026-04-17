#include <ros/ros.h>
#include <robot_sdk_wrapper/robot_sdk.h>

#include <chrono>
#include <thread>

const std::string DEFAULT_ROBOT_IP = "192.168.1.199";
const std::string DEFAULT_PC_IP = "192.168.1.150";
const std::string external_control_file_address =
    "/home/duefor/climbrobot_mjk/src/robot_sdk_wrapper/resource/external_control.script";
const std::string output_recipe_file_address =
    "/home/duefor/climbrobot_mjk/src/robot_sdk_wrapper/resource/output_recipe.txt";
const std::string input_recipe_file_address =
    "/home/duefor/climbrobot_mjk/src/robot_sdk_wrapper/resource/input_recipe.txt";
const std::string task_file_address = "mjktest.task";

// 控制目标参数
double Fz_target = 10.0;    // 期望的z轴末端力(N)
double Md_z = 1.0;          // 虚拟质量
double Bd_z = 10.0;         // 虚拟阻尼
double Kd_z = 50.0;         // 虚拟刚度
double ctrl_period = 0.01;  // 控制周期(s)
double move_distance = 0.2; // 机械臂x方向期望总移动距离(m)
double move_speed = 0.02;   // x方向移动速度(m/s)

int main(int argc, char** argv)
{
    ros::init(argc, argv, "force_test_node");

    EliteCSRobotSDK cs66robot(DEFAULT_ROBOT_IP, DEFAULT_PC_IP, true, external_control_file_address,
                              output_recipe_file_address, input_recipe_file_address, task_file_address, 250);

    if (!cs66robot.init()) {
        ROS_ERROR("Robot init failed.");
        return 1;
    }
    if (!cs66robot.start()) {
        ROS_ERROR("Robot start failed.");
        return 1;
    }

    ELITE::vector6d_t tcp_now = cs66robot.getCurrentTCPPose();
    double start_x = tcp_now[0];
    double start_z = tcp_now[2];
    double dz_dot = 0.0;

    ROS_INFO("Admittance force control test starts (SDK direct mode).");

    const double total_time = move_distance / move_speed;
    double t = 0.0;
    ros::Rate rate(1.0 / ctrl_period);

    while (ros::ok() && t < total_time) {
        tcp_now = cs66robot.getCurrentTCPPose(); // [x,y,z,rx,ry,rz], 姿态为欧拉角
        ELITE::vector6d_t wrench_now = cs66robot.getTCPforce();
        double Fz_meas = wrench_now[2];

        // x方向随时间匀速移动，z方向采用导纳控制
        double cmd_x = start_x + move_speed * t;
        double force_error = Fz_target - Fz_meas;
        double dz_dot_new =
            dz_dot + (ctrl_period / Md_z) * (force_error - Bd_z * dz_dot - Kd_z * (tcp_now[2] - start_z));
        double cmd_z = tcp_now[2] + dz_dot_new * ctrl_period;
        dz_dot = dz_dot_new;

        ELITE::vector6d_t tcp_cmd = tcp_now;
        tcp_cmd[0] = cmd_x;
        tcp_cmd[2] = cmd_z;

        // 笛卡尔伺服控制（cartesian=true）
        if (!cs66robot.writeservoj(tcp_cmd, 100, true, false)) {
            ROS_ERROR("writeservoj failed, stop control loop.");
            break;
        }

        ROS_INFO("t=%.2f x=%.3f z=%.3f Fz=%.2f N err=%.2f", t, cmd_x, cmd_z, Fz_meas, force_error);

        rate.sleep();
        t += ctrl_period;
    }

    cs66robot.disconnect();
    ROS_INFO("Admittance force control test finished.");
    return 0;
}