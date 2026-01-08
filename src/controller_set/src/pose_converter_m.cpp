#include <ros/ros.h>
#include <Eigen/Dense>
#include <algorithm>
#include <robot_set/TCPState.h>
#include <iostream>

using Eigen::Vector3d;

// 防止在极限位置除 0
const double EPS = 1e-6;

// 手柄初始位置，极限位置
Vector3d H_INIT(0.0018713378906249911,  0.0, -0.06545825958251954);
Vector3d H_MAX( 0.18,  0.21,  0.18);
Vector3d H_MIN(0.0018713378906249911, -0.21, -0.1);

// 机械臂初始位置，极限位置
Vector3d R_INIT(0.28, -0.15, 0.28);
Vector3d R_MAX(0.63,  0.26,  0.65);
Vector3d R_MIN(0.28, -0.63,  0.18);

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


    robot_set::TCPState out;
    out.position.resize(6);
    out.velocity.resize(6);

    // position
    out.position[0] = r_pos[0];
    out.position[1] = r_pos[1];
    out.position[2] = r_pos[2];
    // out.position[3] = 3.14;
    // out.position[4] = 0.0;
    // out.position[5] = 0.0;
    out.position[3] = h_rot[0];
    out.position[4] = h_rot[1];
    out.position[5] = h_rot[2];

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

    ros::Subscriber haptic_sub = nh.subscribe("/phantom/pose", 10, hapticCallback);

    robot_pub = nh.advertise<robot_set::TCPState>("/pose_converter/desired_robot_sub", 10);

    ROS_INFO("Haptic to robot mapping node started.");
    ros::spin();
    return 0;
}
