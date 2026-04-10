#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <robot_set/TCPState.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

// 当前 TCP：TCPState.position 为 [x,y,z,rx,ry,rz]，姿态为 XYZ 欧拉角 (rad)
// 期望 TCP：同上数组，但 rx,ry,rz 为旋转向量 (rad，轴角向量)
// 发布：TCPState.position 为 [x,y,z,rx,ry,rz]，姿态为欧拉角 (rad)，与反馈一致
robot_set::TCPState current_tcp_state;
robot_set::TCPState desired_tcp_state;
geometry_msgs::WrenchStamped current_tcp_wrench;

bool got_current_pose = false;
bool got_desired_pose = false;
bool got_current_wrench = false;

const double Fz_target = 10.0;

const double Md_t = 1.0;
const double Bd_t = 20.0;
const double Kd_t = 50.0;

const double Md_r = 0.3;
const double Bd_r = 2.0;
const double Kd_r = 5.0;

const bool kWrenchInToolFrame = true;

ros::Publisher tcp_ctrl_pub;

// 期望位姿：旋转向量 → R（与 pose_error_controller_m 一致）
static Matrix3d rotvecToRot(const Vector3d& r)
{
    double theta = r.norm();
    if (theta < 1e-8)
        return Matrix3d::Identity();
    return AngleAxisd(theta, r / theta).toRotationMatrix();
}

// 当前位姿：固定轴 XYZ 欧拉角 → R（与 pose_error_controller_m 一致）
static Matrix3d eulerXYZToRot(double rx, double ry, double rz)
{
    AngleAxisd Rx(rx, Vector3d::UnitX());
    AngleAxisd Ry(ry, Vector3d::UnitY());
    AngleAxisd Rz(rz, Vector3d::UnitZ());
    return (Rz * Ry * Rx).toRotationMatrix();
}

// R → 欧拉角（ZYX 提取，与 ik_position_solver_m 一致，用于发布）
static void rotToEulerXYZ(const Matrix3d& R, double* rx, double* ry, double* rz)
{
    double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
    bool singular = sy < 1e-6;
    if (!singular) {
        *rx = atan2(R(2, 1), R(2, 2));
        *ry = atan2(-R(2, 0), sy);
        *rz = atan2(R(1, 0), R(0, 0));
    } else {
        *rx = atan2(-R(1, 2), R(1, 1));
        *ry = atan2(-R(2, 0), sy);
        *rz = 0.0;
    }
}

static bool tcpPoseOk(const robot_set::TCPState& s)
{
    return s.position.size() >= 6;
}

void currentTcpPoseCallback(const robot_set::TCPState::ConstPtr& msg)
{
    current_tcp_state = *msg;
    got_current_pose = tcpPoseOk(*msg);
}

void desiredTcpPoseCallback(const robot_set::TCPState::ConstPtr& msg)
{
    desired_tcp_state = *msg;
    got_desired_pose = tcpPoseOk(*msg);
}

void tcpWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    current_tcp_wrench = *msg;
    got_current_wrench = true;
}

void admittanceControl()
{
    static Vector3d v_lin = Vector3d::Zero();
    static Vector3d v_ang = Vector3d::Zero();
    static ros::Time last_time = ros::Time::now();

    if (!got_current_pose || !got_desired_pose || !got_current_wrench)
        return;

    ros::Time now = ros::Time::now();
    double dt = (now - last_time).toSec();
    if (dt < 1e-4)
        return;
    last_time = now;

    Vector3d F_meas(current_tcp_wrench.wrench.force.x, current_tcp_wrench.wrench.force.y,
                    current_tcp_wrench.wrench.force.z);
    Vector3d T_meas(current_tcp_wrench.wrench.torque.x, current_tcp_wrench.wrench.torque.y,
                    current_tcp_wrench.wrench.torque.z);

    const auto& pc = current_tcp_state.position;
    const auto& pd = desired_tcp_state.position;

    Matrix3d R_cur = eulerXYZToRot(pc[3], pc[4], pc[5]);
    Quaterniond q_cur(R_cur);
    q_cur.normalize();

    if (kWrenchInToolFrame) {
        F_meas = R_cur * F_meas;
        T_meas = R_cur * T_meas;
    }

    Vector3d F_des(0.0, 0.0, Fz_target);
    Vector3d T_des = Vector3d::Zero();
    Vector3d e_F = F_des - F_meas;
    Vector3d e_T = T_des - T_meas;

    Vector3d p_cur(pc[0], pc[1], pc[2]);
    Vector3d p_des(pd[0], pd[1], pd[2]);

    Vector3d v_lin_dot = (e_F - Bd_t * v_lin - Kd_t * (p_des - p_cur)) / Md_t;
    v_lin += v_lin_dot * dt;

    Matrix3d R_des = rotvecToRot(Vector3d(pd[3], pd[4], pd[5]));
    Quaterniond q_des(R_des);
    q_des.normalize();
    Quaterniond q_err = q_des * q_cur.inverse();
    q_err.normalize();
    AngleAxisd aa_err(q_err);
    Vector3d theta_err = aa_err.angle() * aa_err.axis();

    Vector3d v_ang_dot = (e_T - Bd_r * v_ang - Kd_r * theta_err) / Md_r;
    v_ang += v_ang_dot * dt;

    robot_set::TCPState ctrl = current_tcp_state;
    ctrl.header.stamp = ros::Time::now();
    ctrl.position.resize(6);

    ctrl.position[0] = pc[0] + v_lin.x() * dt;
    ctrl.position[1] = pc[1] + v_lin.y() * dt;
    ctrl.position[2] = pc[2] + v_lin.z() * dt;

    double wn = v_ang.norm();
    Quaterniond q_new = q_cur;
    if (wn > 1e-8) {
        Quaterniond dq(AngleAxisd(wn * dt, v_ang / wn));
        q_new = (dq * q_cur).normalized();
    }
    Matrix3d R_new = q_new.toRotationMatrix();
    rotToEulerXYZ(R_new, &ctrl.position[3], &ctrl.position[4], &ctrl.position[5]);

    ctrl.velocity.resize(6);
    for (int i = 0; i < 6; ++i)
        ctrl.velocity[i] = 0.0;

    tcp_ctrl_pub.publish(ctrl);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "force_admittance_controller");
    ros::NodeHandle nh;

    ros::Subscriber sub_current_tcp_pose = nh.subscribe("/robot/tcp_pose", 1, currentTcpPoseCallback);
    ros::Subscriber sub_desired_tcp_pose = nh.subscribe("/robot/desired_tcp_pose", 1, desiredTcpPoseCallback);
    ros::Subscriber sub_tcp_wrench = nh.subscribe("/robot/tcp_wrench", 1, tcpWrenchCallback);

    tcp_ctrl_pub = nh.advertise<robot_set::TCPState>("/robot/tcp_ctrl_cmd", 1);

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        admittanceControl();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
