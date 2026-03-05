#include <ros/ros.h>
// #include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <mutex>
#include <robot_set/TCPState.h>
#include <iostream>
#include <iomanip>

using namespace Eigen;

// 临时pid参数
double Kp_lin_n;
double Ki_lin_n;
double Kd_lin_n;
Matrix3d Kp_lin;
Matrix3d Ki_lin;
Matrix3d Kd_lin;

double Kp_rot_n;
double Kvir_rot_n;
Matrix3d Kp_rot;
Matrix3d Kvir_rot;

double beta = 0.0;  // 误差权重参数
double Equal1;
double Equal2;

VectorXd desired_pose(6); // 期望tcp位姿
VectorXd desired_vel(6);  // 期望tcp速度
VectorXd error_prev(6);
VectorXd error_integral(6);
// MatrixXd K_p = MatrixXd::Identity(6,6); // 位姿误差增益矩阵：p
// MatrixXd K_i = MatrixXd::Identity(6,6); // 位姿积分误差增益矩阵：i
// MatrixXd K_d = MatrixXd::Identity(6,6); // 位姿微分误差增益矩阵：d
// 数据锁，防止数据争夺
std::mutex mtx;

// 同步/超时保护
bool desired_valid = false;
ros::Time t_desired;
// 定义超时最长时间
const double TIMEOUT = 0.1; // 100 ms

// 虚拟角速度
Matrix3d R_d_prev = Matrix3d::Identity();
bool R_d_init = false;
Vector3d virtual_rotVel = Vector3d::Zero();

ros::Subscriber sub_actual;
ros::Subscriber sub_desired;
ros::Publisher pub_cmd;

// 期望位姿（旋转向量 → R）
Matrix3d rotvecToRot(const Vector3d& r)
{
  double theta = r.norm();
  if (theta < 1e-8)
    return Matrix3d::Identity();
  return AngleAxisd(theta, r / theta).toRotationMatrix();
}

// 实际位姿（XYZ 欧拉角 → R）
Matrix3d eulerXYZToRot(double rx, double ry, double rz)
{
  AngleAxisd Rx(rx, Vector3d::UnitX());
  AngleAxisd Ry(ry, Vector3d::UnitY());
  AngleAxisd Rz(rz, Vector3d::UnitZ());
  return (Rz * Ry * Rx).toRotationMatrix();   // 固定轴 XYZ
}


// Matrix4d poseToHomog(const VectorXd &p)
// {
//   Matrix4d T = Matrix4d::Identity();
//   T.block<3,3>(0,0) = rotvecToRot(p[3], p[4], p[5]);
//   T.block<3,1>(0,3) = p.head<3>();
//   return T;
// }


// 李代数误差公式，求出位姿误差向量
VectorXd se3Log(const Matrix4d &E)
{
  Vector3d omega;
  Matrix3d R = E.block<3,3>(0,0);
  double theta = acos(std::min(1.0, std::max(-1.0, (R.trace()-1)/2)));
  if (fabs(theta) < 1e-6)
    omega.setZero();
  else
    omega = theta/(2*sin(theta))*Vector3d(R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1));

  Matrix3d Omega = Matrix3d::Zero();
  Omega(0,1) = -omega(2); Omega(0,2) = omega(1);
  Omega(1,0) = omega(2);  Omega(1,2) = -omega(0);
  Omega(2,0) = -omega(1); Omega(2,1) = omega(0);

  Matrix3d I = Matrix3d::Identity();
  Matrix3d V;
  if (fabs(theta) < 1e-6)
    V = I;
  else
    V = I + (1 - cos(theta))/pow(theta,2)*Omega + (theta - sin(theta))/pow(theta,3)*(Omega*Omega);

  Vector3d v = V.inverse() * E.block<3,1>(0,3);
  VectorXd xi(6);
  xi << v, omega;
  return xi;
}


// 期望的机械臂tcp位姿和速度
void desiredCB(const robot_set::TCPState::ConstPtr &msg)
{
  if (msg->position.size() == 6 && msg->velocity.size() == 6)
  {
    std::lock_guard<std::mutex> lk(mtx);
    for (int i=0;i<6;i++) desired_pose[i]=msg->position[i];
    for (int i=0;i<6;i++) desired_vel[i]=msg->velocity[i];
    desired_valid = true;
    // 不在意同步，只在意安全性，故不用stamp而是直接使用接收信息的时间。
    t_desired = ros::Time::now();
  }
}


// 机械臂实际tcp位姿反馈
void actualCB(const robot_set::TCPState::ConstPtr &msg)
{
  if (msg->position.size() != 6) return;

  static ros::Time last_ctl_time;
  // 同步/超时保护
  ros::Time now = ros::Time::now();
  if(last_ctl_time.isZero())
  {
    last_ctl_time = now;
    return;
  }

  bool ready = desired_valid && (now - t_desired).toSec() < TIMEOUT ;

  // 如果没接收到期望的机械臂位姿和速度或是突然断开
  if (!ready)
  {
    error_integral.setZero();
    error_prev.setZero();
    last_ctl_time = now;
    R_d_init = false;
    beta = 0.0;
    return;
  }

  VectorXd a(6), d, v_d;
  for (int i=0;i<6;i++) a[i]=msg->position[i];
  {
    std::lock_guard<std::mutex> lk(mtx);
    d = desired_pose;
    v_d = desired_vel;
  }

  Matrix4d T_d = Matrix4d::Identity();
  T_d.block<3,3>(0,0) = rotvecToRot(d.tail<3>());
  T_d.block<3,1>(0,3) = d.head<3>();
  Matrix4d T_a = Matrix4d::Identity();
  T_a.block<3,3>(0,0) = eulerXYZToRot(a[3], a[4], a[5]);
  T_a.block<3,1>(0,3) = a.head<3>();

  Matrix4d E = T_d * T_a.inverse();
  VectorXd debug_print = se3Log(E);


  VectorXd xi(6);
  xi.head<3>() = T_d.block<3,1>(0,3) - T_a.block<3,1>(0,3);   // 位置误差直接相减得到
  
  auto xi_rot = T_a.block<3,3>(0,0).transpose() * T_d.block<3,3>(0,0);  // 自身坐标系中的姿态误差
  xi.tail<3>() = AngleAxisd(xi_rot).axis() * AngleAxisd(xi_rot).angle();   // 姿态误差
  
  double dt = (now - last_ctl_time).toSec();
  last_ctl_time = now;
  if (dt <= 1e-6 || dt > 0.05) return;
  
  ///////////////////////////////////////////////////////
  // 机械臂权重跟随，防止机械臂初始化时失控
  double e_norm = xi.head<3>().norm();   // 只用平移误差
  std::cout << "平移误差为：" << e_norm << std::endl;

  if(beta < 0.99) {
    double beta_target;
    if (e_norm <= Equal1)
      beta_target = 1.0;
    else if (e_norm >= Equal2)
      beta_target = 0.0;
    else
      beta_target = (Equal2 - e_norm) / (Equal2 - Equal1);

    double tau = 0.3;   // 对齐时间常数（秒，0.2~0.5 很合适）
    double beta_dot = (beta_target - beta) / tau;
    beta += beta_dot * dt;
    beta = std::clamp(beta, 0.0, 1.0);
  }
  else {
    beta = 1.0; // 机械臂已经可以合理跟随手柄，将权重置1
  }
  std::cout << "当前跟随权重：" << beta << std::endl;
  //////////////////////////////////////////////////////

  //////////////////////////////////////////////////////
  // 速度前馈权重
  static double alpha;   // 前馈权重（0~1）=
  static double E_FF_ON  = 0.15;   // 误差大于这个，前馈全开

  if (e_norm >= E_FF_ON)
    alpha = 1.0;
  else
    alpha = (e_norm) / (E_FF_ON);
  //////////////////////////////////////////////////////

  // std::cout<<xi<<std::endl;
  // std::cout<<"-----------------------------------------"<<std::endl;
  // std::cout << "当前位姿指令为：" << "[" << a[0] << "," << a[1] << "," << a[2] << "," 
  //     << a[3] << "," << a[4] << "," << a[5] << "]" << std::endl;

  // auto xii = debug_print;
  // for(int i = 0; i<6; ++i)
  // {
  //   xii[i]=std::round(xii[i]*1e7)/1e7;
  // }
  // std::cout<<std::fixed<<std::setprecision(7);
  // std::cout<<"原始误差:"<<std::endl<<xii<<std::endl;
  // std::cout<<"-----------------------------------------"<<std::endl;
  

  // 积分项（只对平移）
  error_integral.head<3>() += xi.head<3>() * dt;
  error_integral.tail<3>().setZero();

  double integral_limit = 2.0;
  for (int i = 0; i < 3; ++i)
  {
    error_integral[i] = std::clamp(error_integral[i], -integral_limit, integral_limit);
  }

  // 微分项（只对平移）
  VectorXd error_derivative = VectorXd::Zero(6);
  error_derivative.head<3>() =
      (xi.head<3>() - error_prev.head<3>()) / dt;

  error_prev = xi;

  // PID 控制律
  VectorXd v_cmd = VectorXd::Zero(6);
  // 平移：PID
  v_cmd.head<3>() =
      beta * (alpha * v_d.head<3>()
    + Kp_lin * xi.head<3>()
    + Ki_lin * error_integral.head<3>()
    + Kd_lin * error_derivative.head<3>());

  // 制造虚拟角速度（tcp空间），因为touch无法发布角速度
  if (R_d_init)
  {
    Matrix3d R_delta = R_d_prev.transpose() * T_d.block<3,3>(0,0);
    AngleAxisd aa(R_delta);
    virtual_rotVel = aa.axis() * aa.angle() / dt;
  }
  else
  {
    R_d_init = true;
  }

  R_d_prev = T_d.block<3,3>(0,0);
  

  // 姿态：虚拟加速度 + P
  v_cmd.tail<3>() = beta * (Kvir_rot * virtual_rotVel +
      Kp_rot * xi.tail<3>());

  // // --- 姿态误差 ---
  // Matrix3d R_a = T_a.block<3,3>(0,0);
  // Matrix3d R_d = T_d.block<3,3>(0,0);
  // AngleAxisd aa(R_d * R_a.transpose());
  // Vector3d e_w = aa.axis() * aa.angle();
  // static Vector3d e_w_prev = Vector3d::Zero();
  // Vector3d e_w_dot = (e_w - e_w_prev) / dt;
  // e_w_prev = e_w;
  // v_cmd.tail<3>() =
  //     Kp_rot * e_w
  //   - Kvir_rot * e_w_dot;


  robot_set::TCPState tcp_vel_msg;
  tcp_vel_msg.velocity.resize(6);
  tcp_vel_msg.header.stamp = ros::Time::now();
  for (int i=0;i<6;i++) tcp_vel_msg.velocity[i]=v_cmd[i];
  pub_cmd.publish(tcp_vel_msg);
  // std::cout << "当前tcp速度控制指令为：" << "[" << tcp_vel_msg.velocity[0] << "," << tcp_vel_msg.velocity[1] << "," << tcp_vel_msg.velocity[2] << "," 
  //     << tcp_vel_msg.velocity[3] << "," << tcp_vel_msg.velocity[4] << "," << tcp_vel_msg.velocity[5] << "]" << std::endl;
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "pose_error_controller");
  ros::NodeHandle nh;

  ros::param::param(std::string("~KP_LIN_N"), Kp_lin_n, double(2.5));
  ros::param::param(std::string("~KI_LIN_N"), Ki_lin_n, double(0.01));
  ros::param::param(std::string("~KD_LIN_N"), Kd_lin_n, double(0.3));

  ros::param::param(std::string("~KP_ROT_N"), Kp_rot_n, double(1.5));
  ros::param::param(std::string("~KVIR_ROT_N"), Kvir_rot_n, double(1.0));

  // 对齐阈值，tcp末端与期望末端的距离（误差）
  ros::param::param(std::string("~EQUAL1"), Equal1, double(0.02));
  ros::param::param(std::string("~EQUAL2"), Equal2, double(0.43));

  Kp_lin = Kp_lin_n * Matrix3d::Identity();
  Ki_lin = Ki_lin_n * Matrix3d::Identity();
  Kd_lin = Kd_lin_n * Matrix3d::Identity();

  Kp_rot = Kp_rot_n * Matrix3d::Identity();
  Kvir_rot = Kvir_rot_n * Matrix3d::Identity();

  desired_pose.setZero();
  desired_vel.setZero();

  error_prev.setZero();
  error_integral.setZero();

  sub_actual = nh.subscribe("/cs66/tcp_state", 1, actualCB);
  sub_desired = nh.subscribe("/pose_converter/desired_robot_sub", 1, desiredCB);
  pub_cmd = nh.advertise<robot_set::TCPState>("/controller/cartesian_vel", 1);


  ros::spin();
  return 0;
}