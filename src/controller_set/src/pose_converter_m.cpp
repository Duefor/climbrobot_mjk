#include <ros/ros.h>
#include <Eigen/Dense>
#include <algorithm>
#include <robot_set/TCPState.h>
#include <iostream>

using Eigen::Vector3d;

// 防止在极限位置除 0
const double EPS = 1e-6;

// 手柄初始位置，极限位置
Vector3d H_INIT;
Vector3d H_MAX;
Vector3d H_MIN;

// 机械臂初始位置，极限位置
Vector3d R_INIT;
Vector3d R_MAX;
Vector3d R_MIN;

int CONTROL_MODE;

ros::Publisher robot_pub;

double clamp(double v, double vmin, double vmax)
{
    return std::max(vmin, std::min(v, vmax));
}

// 位置映射
double mapAxis(
    double h, double h0,
    double hmin, double hmax,
    double r0, double rmin, double rmax)
{
    double dh = h - h0;
    double gain;

    if (dh >= 0.0)
        gain = (rmax - r0) / std::max(hmax - h0, EPS);
    else
        gain = (r0 - rmin) / std::max(h0 - hmin, EPS);

    return clamp(r0 + gain * dh, rmin, rmax);
}

// 速度映射（仅比例）
double mapVelocityAxis(
    double v,
    double h0,
    double hmin, double hmax,
    double r0, double rmin, double rmax)
{
    double gain_pos = (rmax - r0) / std::max(hmax - h0, EPS);
    double gain_neg = (r0 - rmin) / std::max(h0 - hmin, EPS);

    double gain = (v >= 0.0) ? gain_pos : gain_neg;
    return gain * v;
}



void hapticCallback(const robot_set::TCPState::ConstPtr& msg)
{
    if (msg->position.size() < 6 || msg->velocity.size() < 6) return;

    // ////////////////////////////////////////////////////
    // // 一阶低通滤波
    // static bool haptic_initialized = false;
    // static Vector3d h_pos_filt = Vector3d::Zero();
    // static Vector3d h_vel_filt = Vector3d::Zero();
    // static ros::Time last_time;
    // ros::Time now = ros::Time::now();

    // if (!haptic_initialized)
    // {
    //     h_pos_filt << msg->position[0], msg->position[1], msg->position[2];
    //     h_vel_filt << msg->velocity[0], msg->velocity[1], msg->velocity[2];
    //     last_time = now;
    //     haptic_initialized = true;
    // }

    // double dt = (now - last_time).toSec();
    // last_time = now;
    // if (dt < 1e-4) dt = 1e-4;

    // // ===== 一阶低通参数 =====
    // double fc_pos = 4.0;   // 位置截止频率（Hz）
    // double fc_vel = 6.0;   // 速度截止频率（Hz）

    // double alpha_p = 2.0 * M_PI * fc_pos * dt /
    //                 (1.0 + 2.0 * M_PI * fc_pos * dt);
    // double alpha_v = 2.0 * M_PI * fc_vel * dt /
    //                 (1.0 + 2.0 * M_PI * fc_vel * dt);

    // Vector3d h_pos_raw(msg->position[0], msg->position[1], msg->position[2]);
    // Vector3d h_vel_raw(msg->velocity[0], msg->velocity[1], msg->velocity[2]);

    // h_pos_filt += alpha_p * (h_pos_raw - h_pos_filt);
    // h_vel_filt += alpha_v * (h_vel_raw - h_vel_filt);

    // Vector3d h_pos = h_pos_filt;
    // Vector3d h_vel = h_vel_filt;

    // ////////////////////////////////////////////////////

    Vector3d h_pos(msg->position[0], msg->position[1], msg->position[2]);
    Vector3d h_rot(msg->position[3], msg->position[4], msg->position[5]);

    Vector3d h_vel(msg->velocity[0], msg->velocity[1], msg->velocity[2]);

    Vector3d r_pos, r_vel;

    for (int i = 0; i < 3; ++i)
    {
        r_pos[i] = mapAxis(
            h_pos[i], H_INIT[i],
            H_MIN[i],  H_MAX[i],
            R_INIT[i], R_MIN[i], R_MAX[i]);

        r_vel[i] = mapVelocityAxis(
            h_vel[i],
            H_INIT[i],
            H_MIN[i],  H_MAX[i],
            R_INIT[i], R_MIN[i], R_MAX[i]);
    }

    // //////////////////////////////////////////////////////////
    // // 速度平滑器
    // static bool vel_initialized = false;
    // static Vector3d v_state = Vector3d::Zero();
    // static ros::Time last_time_1;

    // ros::Time now_1 = ros::Time::now();
    // if (!vel_initialized)
    // {
    //     v_state = r_vel;
    //     last_time_1 = now_1;
    //     vel_initialized = true;
    // }

    // double dt_1 = (now_1 - last_time_1).toSec();
    // last_time_1 = now_1;
    // if (dt_1 < 1e-4) dt_1 = 1e-4;

    // // ===== 虚拟动力学参数 =====
    // double M = 1.0;   // 虚拟质量（越小越灵）
    // double D = 8.0;   // 阻尼（6~12 推荐）

    // // 一阶惯性-阻尼速度模型
    // v_state += (r_vel - v_state) * (D / M) * dt_1;

    // // 使用平滑后的速度
    // r_vel = v_state;
    // //////////////////////////////////////////////////////////


    robot_set::TCPState out;
    out.position.resize(6);
    out.velocity.resize(6);

    // position
    out.position[0] = r_pos[0];
    out.position[1] = r_pos[1];
    out.position[2] = r_pos[2];
    switch (CONTROL_MODE) {
        case 0:
            out.position[3] = 3.14;
            out.position[4] = 0.0;
            out.position[5] = 0.0;
            break;
        case 1:
            out.position[3] = h_rot[0];
            out.position[4] = h_rot[1];
            out.position[5] = h_rot[2];
            break;
        default:
            out.position[3] = 3.14;
            out.position[4] = 0.0;
            out.position[5] = 0.0;
            break;
    }

    // velocity（只映射 xyz，其余清零）
    out.velocity[0] = r_vel[0];
    out.velocity[1] = r_vel[1];
    out.velocity[2] = r_vel[2];
    out.velocity[3] = 0.0;
    out.velocity[4] = 0.0;
    out.velocity[5] = 0.0;

    out.header.stamp = ros::Time::now();

    robot_pub.publish(out);
}

// ---------------- main ----------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "haptic_to_robot_mapping");
    ros::NodeHandle nh;

    // 手柄初始位姿
    for (int i = 0; i < 3; i++) {
        double default_val = 0.0018713378906249911;
        if (i == 1) default_val = 0.0;
        if (i == 2) default_val = -0.06545825958251954;
        ros::param::param(std::string("~H_INIT_") + std::to_string(i+1), H_INIT(i), default_val);
    }
    // 手柄极限最大位姿
    for (int i = 0; i < 3; i++) {
        double default_val = 0.18;
        if (i == 1) default_val = 0.21;
        if (i == 2) default_val = 0.18;
        ros::param::param(std::string("~H_MAX_") + std::to_string(i+1), H_MAX(i), default_val);
    }
    // 手柄极限最小位姿
    for (int i = 0; i < 3; i++) {
        double default_val = 0.0018713378906249911;
        if (i == 1) default_val = -0.21;
        if (i == 2) default_val = -0.1;
        ros::param::param(std::string("~H_MIN_") + std::to_string(i+1), H_MIN(i), default_val);
    }
    // 机械臂初始位姿
    for (int i = 0; i < 3; i++) {
        double default_val = 0.28;
        if (i == 1) default_val = -0.15;
        if (i == 2) default_val = 0.28;
        ros::param::param(std::string("~R_INIT_") + std::to_string(i+1), R_INIT(i), default_val);
    }
    // 机械臂极限最大位姿
    for (int i = 0; i < 3; i++) {
        double default_val = 0.63;
        if (i == 1) default_val = 0.26;
        if (i == 2) default_val = 0.706;
        ros::param::param(std::string("~R_MAX_") + std::to_string(i+1), R_MAX(i), default_val);
    }
    // 机械臂极限最小位姿
    for (int i = 0; i < 3; i++) {
        double default_val = 0.28;
        if (i == 1) default_val = -0.56;
        if (i == 2) default_val = 0.22;
        ros::param::param(std::string("~R_MIN_") + std::to_string(i+1), R_MIN(i), default_val);
    }
    // 映射模式
    ros::param::param(std::string("~CONTROL_MODE"), CONTROL_MODE, int(0));


    ros::Subscriber haptic_sub = nh.subscribe("/phantom/pose", 10, hapticCallback);

    robot_pub = nh.advertise<robot_set::TCPState>("/pose_converter/desired_robot_sub", 10);

    ROS_INFO("Haptic to robot mapping node started.");
    ros::spin();
    return 0;
}
