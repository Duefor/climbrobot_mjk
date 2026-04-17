/**
 * Integrated teleop node: haptic TCP -> workspace mapping -> Cartesian PID + feedforward ->
 * MoveIt Jacobian velocity IK -> Elite SDK joint velocity.
 */
#include <array>
#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <robot_set/TCPState.h>
#include <robot_sdk_wrapper/robot_sdk.h>

// 全局常量
constexpr double kEps = 1e-6;
constexpr double kPi = 3.14159265358979323846;

// 将数值限制到指定范围内，常用于速度、角度等限制。
inline double clampScalar(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

// 将 XYZ 欧拉角转换为 TCP 末端的旋转矩阵。
// 这里采用 ZYX 顺序，即先绕 X 旋转，再绕 Y，最后绕 Z。
inline Eigen::Matrix3d eulerXYZToRotForTcp(double rx, double ry, double rz) {
  Eigen::AngleAxisd Rx(rx, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd Ry(ry, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Rz(rz, Eigen::Vector3d::UnitZ());
  return (Rz * Ry * Rx).toRotationMatrix();
}

// ---- 手柄姿态到机器人工作空间映射 ----
// 该模块将手柄的 TCP 位姿转换为机器人期望的 TCP 位姿，用于后续闭环控制。
// 仅对平移部分做线性比例映射，姿态部分根据 CONTROL_MODE 决定是否传递手柄姿态。
class PoseMapper {
 public:
  void load(ros::NodeHandle& pnh) {
    for (int i = 0; i < 3; ++i) {
      const int j = i + 1;
      pnh.param("H_INIT_" + std::to_string(j), h_init_[i], defaultHInit(i));
      pnh.param("H_MAX_" + std::to_string(j), h_max_[i], defaultHMax(i));
      pnh.param("H_MIN_" + std::to_string(j), h_min_[i], defaultHMin(i));
      pnh.param("R_INIT_" + std::to_string(j), r_init_[i], defaultRInit(i));
      pnh.param("R_MAX_" + std::to_string(j), r_max_[i], defaultRMax(i));
      pnh.param("R_MIN_" + std::to_string(j), r_min_[i], defaultRMin(i));
    }
    pnh.param("CONTROL_MODE", control_mode_, 0);
  }

  void map(const robot_set::TCPState& in, robot_set::TCPState& out) const {
    if (in.position.size() < 6 || in.velocity.size() < 6) {
      return;
    }
    Eigen::Vector3d h_pos(in.position[0], in.position[1], in.position[2]);
    Eigen::Vector3d h_rot(in.position[3], in.position[4], in.position[5]);
    Eigen::Vector3d h_vel(in.velocity[0], in.velocity[1], in.velocity[2]);
    Eigen::Vector3d r_pos;
    Eigen::Vector3d r_vel;
    for (int i = 0; i < 3; ++i) {
      r_pos[i] = mapAxis(h_pos[i], h_init_[i], h_min_[i], h_max_[i], r_init_[i], r_min_[i], r_max_[i]);
      r_vel[i] = mapVelocityAxis(h_vel[i], h_init_[i], h_min_[i], h_max_[i], r_init_[i], r_min_[i], r_max_[i]);
    }
    out.position.resize(6);
    out.velocity.resize(6);
    out.position[0] = r_pos[0];
    out.position[1] = r_pos[1];
    out.position[2] = r_pos[2];
    if (control_mode_ == 1) {
      out.position[3] = h_rot[0];
      out.position[4] = h_rot[1];
      out.position[5] = h_rot[2];
    } else {
      out.position[3] = kPi;
      out.position[4] = 0.0;
      out.position[5] = 0.0;
    }
    out.velocity[0] = r_vel[0];
    out.velocity[1] = r_vel[1];
    out.velocity[2] = r_vel[2];
    out.velocity[3] = out.velocity[4] = out.velocity[5] = 0.0;
    out.header.stamp = ros::Time::now();
  }

 private:
  static double defaultHInit(int i) {
    static const double d[3] = {0.0018713378906249911, 0.0, -0.06545825958251954};
    return d[i];
  }
  static double defaultHMax(int i) {
    static const double d[3] = {0.18, 0.21, 0.18};
    return d[i];
  }
  static double defaultHMin(int i) {
    static const double d[3] = {0.0018713378906249911, -0.21, -0.1};
    return d[i];
  }
  static double defaultRInit(int i) {
    static const double d[3] = {0.28, -0.15, 0.28};
    return d[i];
  }
  static double defaultRMax(int i) {
    static const double d[3] = {0.63, 0.26, 0.706};
    return d[i];
  }
  static double defaultRMin(int i) {
    static const double d[3] = {0.28, -0.56, 0.22};
    return d[i];
  }

  static double mapAxis(double h, double h0, double hmin, double hmax, double r0, double rmin, double rmax) {
    const double dh = h - h0;
    const double gain_pos = (rmax - r0) / std::max(hmax - h0, kEps);
    const double gain_neg = (r0 - rmin) / std::max(h0 - hmin, kEps);
    const double gain = (dh >= 0.0) ? gain_pos : gain_neg;
    return clampScalar(r0 + gain * dh, rmin, rmax);
  }

  static double mapVelocityAxis(double v, double h0, double hmin, double hmax, double r0, double rmin, double rmax) {
    const double gain_pos = (rmax - r0) / std::max(hmax - h0, kEps);
    const double gain_neg = (r0 - rmin) / std::max(h0 - hmin, kEps);
    const double gain = (v >= 0.0) ? gain_pos : gain_neg;
    return gain * v;
  }

  Eigen::Vector3d h_init_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d h_max_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d h_min_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d r_init_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d r_max_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d r_min_{Eigen::Vector3d::Zero()};
  int control_mode_{0};
};

// ---- 笛卡尔空间 PID + 前馈控制器 ----
// 该模块基于实际 TCP 与期望 TCP 计算跟踪误差，并输出 6 维笛卡尔速度命令。
// 其中：
// - 位置误差使用标准 PID
// - 旋转误差使用虚拟角速度前馈 + 比例控制
// - beta 用于在大误差时逐步引入控制，避免直接强制干预导致运动突变
class PoseErrorController {
 public:
  void load(ros::NodeHandle& pnh) {
    double kp_lin_n = 2.5, ki_lin_n = 0.01, kd_lin_n = 0.3;
    double kp_rot_n = 1.5, kvir_rot_n = 1.0;
    pnh.param("KP_LIN_N", kp_lin_n, kp_lin_n);
    pnh.param("KI_LIN_N", ki_lin_n, ki_lin_n);
    pnh.param("KD_LIN_N", kd_lin_n, kd_lin_n);
    pnh.param("KP_ROT_N", kp_rot_n, kp_rot_n);
    pnh.param("KVIR_ROT_N", kvir_rot_n, kvir_rot_n);
    pnh.param("EQUAL1", equal1_, 0.02);
    pnh.param("EQUAL2", equal2_, 0.43);
    pnh.param("E_FF_ON", e_ff_on_, 0.15);
    pnh.param("integral_limit", integral_limit_, 2.0);
    pnh.param("beta_tau", beta_tau_, 0.3);
    kp_lin_ = kp_lin_n * Eigen::Matrix3d::Identity();
    ki_lin_ = ki_lin_n * Eigen::Matrix3d::Identity();
    kd_lin_ = kd_lin_n * Eigen::Matrix3d::Identity();
    kp_rot_ = kp_rot_n * Eigen::Matrix3d::Identity();
    kvir_rot_ = kvir_rot_n * Eigen::Matrix3d::Identity();
    reset();
  }

  void reset() {
    error_prev_.setZero();
    error_integral_.setZero();
    beta_ = 0.0;
    r_d_prev_ = Eigen::Matrix3d::Identity();
    r_d_init_ = false;
    virtual_rot_vel_.setZero();
    last_ctl_time_valid_ = false;
  }

  Eigen::VectorXd compute(const Eigen::VectorXd& actual6, const Eigen::VectorXd& desired_pose,
                          const Eigen::VectorXd& desired_vel, double now_sec, bool desired_ready,
                          double desired_age_sec) {
    Eigen::VectorXd v_cmd = Eigen::VectorXd::Zero(6);
    if (!last_ctl_time_valid_) {
      last_ctl_time_ = now_sec;
      last_ctl_time_valid_ = true;
      return v_cmd;
    }
    double dt = now_sec - last_ctl_time_;
    last_ctl_time_ = now_sec;
    if (dt <= 1e-6 || dt > 0.05) {
      return v_cmd;
    }

    if (!desired_ready || desired_age_sec > desired_timeout_) {
      resetPartial();
      return v_cmd;
    }

    Eigen::Matrix4d T_d = Eigen::Matrix4d::Identity();
    T_d.block<3, 3>(0, 0) = rotvecToRot(desired_pose.tail<3>());
    T_d.block<3, 1>(0, 3) = desired_pose.head<3>();
    Eigen::Matrix4d T_a = Eigen::Matrix4d::Identity();
    T_a.block<3, 3>(0, 0) = eulerXYZToRot(actual6[3], actual6[4], actual6[5]);
    T_a.block<3, 1>(0, 3) = actual6.head<3>();

    Eigen::VectorXd xi(6);
    xi.head<3>() = T_d.block<3, 1>(0, 3) - T_a.block<3, 1>(0, 3);
    Eigen::Matrix3d xi_rot = T_a.block<3, 3>(0, 0).transpose() * T_d.block<3, 3>(0, 0);
    Eigen::AngleAxisd aa(xi_rot);
    xi.tail<3>() = aa.axis() * aa.angle();

    const double e_norm = xi.head<3>().norm();

    if (beta_ < 0.99) {
      double beta_target = 0.0;
      if (e_norm <= equal1_) {
        beta_target = 1.0;
      } else if (e_norm >= equal2_) {
        beta_target = 0.0;
      } else {
        beta_target = (equal2_ - e_norm) / (equal2_ - equal1_);
      }
      const double beta_dot = (beta_target - beta_) / beta_tau_;
      beta_ += beta_dot * dt;
      beta_ = clampScalar(beta_, 0.0, 1.0);
    } else {
      beta_ = 1.0;
    }

    double alpha = (e_norm >= e_ff_on_) ? 1.0 : (e_norm / std::max(e_ff_on_, kEps));

    // 仅对位置误差积分，避免旋转误差积分引入复杂的姿态累积.
    error_integral_.head<3>() += xi.head<3>() * dt;
    error_integral_.tail<3>().setZero();
    for (int i = 0; i < 3; ++i) {
      error_integral_[i] = clampScalar(error_integral_[i], -integral_limit_, integral_limit_);
    }

    // 位置误差微分项仅对平移轴计算。旋转部分当前使用虚拟角速度前馈。
    Eigen::VectorXd error_derivative = Eigen::VectorXd::Zero(6);
    error_derivative.head<3>() = (xi.head<3>() - error_prev_.head<3>()) / dt;
    error_prev_ = xi;

    v_cmd.head<3>() = beta_ * (alpha * desired_vel.head<3>() + kp_lin_ * xi.head<3>() +
                               ki_lin_ * error_integral_.head<3>() + kd_lin_ * error_derivative.head<3>());

    if (r_d_init_) {
      Eigen::Matrix3d R_delta = r_d_prev_.transpose() * T_d.block<3, 3>(0, 0);
      Eigen::AngleAxisd aa_d(R_delta);
      virtual_rot_vel_ = aa_d.axis() * aa_d.angle() / dt;
    } else {
      r_d_init_ = true;
    }
    r_d_prev_ = T_d.block<3, 3>(0, 0);
    v_cmd.tail<3>() = beta_ * (kvir_rot_ * virtual_rot_vel_ + kp_rot_ * xi.tail<3>());
    return v_cmd;
  }

  void setDesiredTimeout(double t) { desired_timeout_ = t; }

 private:
  void resetPartial() {
    error_integral_.setZero();
    error_prev_.setZero();
    r_d_init_ = false;
    beta_ = 0.0;
  }

  static Eigen::Matrix3d rotvecToRot(const Eigen::Vector3d& r) {
    const double theta = r.norm();
    if (theta < 1e-8) {
      return Eigen::Matrix3d::Identity();
    }
    return Eigen::AngleAxisd(theta, r / theta).toRotationMatrix();
  }

  static Eigen::Matrix3d eulerXYZToRot(double rx, double ry, double rz) {
    Eigen::AngleAxisd Rx(rx, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd Ry(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rz(rz, Eigen::Vector3d::UnitZ());
    return (Rz * Ry * Rx).toRotationMatrix();
  }

  Eigen::Matrix3d kp_lin_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d ki_lin_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d kd_lin_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d kp_rot_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d kvir_rot_{Eigen::Matrix3d::Identity()};
  double equal1_{0.02};
  double equal2_{0.43};
  double e_ff_on_{0.15};
  double integral_limit_{2.0};
  double beta_tau_{0.3};
  double desired_timeout_{0.1};

  Eigen::VectorXd error_prev_{Eigen::VectorXd::Zero(6)};
  Eigen::VectorXd error_integral_{Eigen::VectorXd::Zero(6)};
  double beta_{0.0};
  Eigen::Matrix3d r_d_prev_{Eigen::Matrix3d::Identity()};
  bool r_d_init_{false};
  Eigen::Vector3d virtual_rot_vel_{Eigen::Vector3d::Zero()};
  double last_ctl_time_{0.0};
  bool last_ctl_time_valid_{false};
};

// ---- MoveIt 雅可比阻尼最小二乘速度 IK ----
// 将笛卡尔速度命令映射为关节速度命令，并在奇异区域添加阻尼。
class MoveItVelocityIk {
 public:
  bool init(const std::string& planning_group, const std::string& tip_link, std::string* err) {
    planning_group_ = planning_group;
    tip_link_ = tip_link;
    try {
      robot_model_loader::RobotModelLoader loader("robot_description");
      robot_model_ = loader.getModel();
    } catch (const std::exception& e) {
      if (err) {
        *err = e.what();
      }
      return false;
    }
    if (!robot_model_) {
      if (err) {
        *err = "robot_model is null";
      }
      return false;
    }
    jmg_ = robot_model_->getJointModelGroup(planning_group_);
    if (!jmg_) {
      if (err) {
        *err = "Unknown planning group: " + planning_group_;
      }
      return false;
    }
    tip_model_ = robot_model_->getLinkModel(tip_link_);
    if (!tip_model_) {
      if (err) {
        *err = "Unknown tip link: " + tip_link_;
      }
      return false;
    }
    const auto& names = jmg_->getActiveJointModelNames();
    if (names.size() != 6) {
      ROS_WARN("MoveItVelocityIk: planning_group has %zu DOF; expected 6.", names.size());
    }
    return true;
  }

  void loadIkParams(ros::NodeHandle& pnh) {
    pnh.param("ik_svd_threshold", svd_thresh_, 0.05);
    pnh.param("ik_damping_gain", damping_gain_, 0.1);
    for (int i = 0; i < 6; ++i) {
      pnh.param("MAX_QDDOT_" + std::to_string(i + 1), max_qddot_[i], 0.6);
      pnh.param("MAX_QDOT_SCALE_" + std::to_string(i + 1), max_qdot_scale_[i], 0.05);
    }
    static constexpr double sdk_max[6] = {5 * kPi / 6, 5 * kPi / 6, kPi, 23 * kPi / 18, 23 * kPi / 18, 23 * kPi / 18};
    for (int i = 0; i < 6; ++i) {
      max_qdot_[i] = sdk_max[i] * max_qdot_scale_[i];
    }
  }

  bool solve(const double q6[6], const Eigen::VectorXd& twist6, Eigen::VectorXd& qdot_out, double dt) {
    if (!robot_model_ || !jmg_ || !tip_model_) {
      return false;
    }
    const unsigned int n = jmg_->getVariableCount();
    Eigen::VectorXd q(n);
    for (unsigned int i = 0; i < n && i < 6u; ++i) {
      q[i] = q6[i];
    }
    for (unsigned int i = 6; i < n; ++i) {
      q[i] = 0.0;
    }

    moveit::core::RobotState rstate(robot_model_);
    rstate.setToDefaultValues();
    rstate.setJointGroupPositions(jmg_, q);
    rstate.update();

    Eigen::MatrixXd J;
    if (!rstate.getJacobian(jmg_, tip_model_, Eigen::Vector3d::Zero(), J)) {
      return false;
    }
    if (J.rows() != 6 || static_cast<unsigned int>(J.cols()) != n) {
      ROS_WARN_THROTTLE(2.0, "Unexpected Jacobian size %ldx%ld", J.rows(), J.cols());
      return false;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double sigma_min = svd.singularValues().size() > 0 ? svd.singularValues().minCoeff() : 1.0;
    double lambda = 0.0;
    if (sigma_min < svd_thresh_) {
      // 奇异值过小时启用阻尼，避免速度解爆炸。
      lambda = damping_gain_ * (svd_thresh_ - sigma_min);
    }
    Eigen::MatrixXd I6 = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd JJT = J * J.transpose();
    Eigen::MatrixXd inv = (JJT + lambda * lambda * I6).inverse();
    Eigen::VectorXd qdot = J.transpose() * inv * twist6;

    double scale = 1.0;
    for (unsigned int i = 0; i < n && i < 6u; ++i) {
      if (std::abs(qdot[i]) > 1e-6) {
        scale = std::min(scale, max_qdot_[i] / std::abs(qdot[i]));
      }
    }
    if (scale < 1.0) {
      qdot *= scale;
    }

    if (!ik_initialized_ || last_qdot_ik_.size() != static_cast<int>(n)) {
      last_qdot_ik_ = Eigen::VectorXd::Zero(n);
      ik_initialized_ = true;
    }
    {
      const double dtm = std::max(dt, 1e-4);
      for (unsigned int i = 0; i < n; ++i) {
        // 对关节速度变化率进行限幅，避免速度突变导致机械臂失稳。
        const double max_delta = (i < 6u ? max_qddot_[i] : max_qddot_[5]) * dtm;
        double delta = qdot[i] - last_qdot_ik_[i];
        delta = clampScalar(delta, -max_delta, max_delta);
        qdot[i] = last_qdot_ik_[i] + delta;
      }
    }
    last_qdot_ik_ = qdot;

    qdot_out.resize(6);
    for (int i = 0; i < 6; ++i) {
      qdot_out[i] = i < q.size() ? qdot[i] : 0.0;
    }
    return true;
  }

  void resetIkState() {
    ik_initialized_ = false;
    last_qdot_ik_.resize(0);
  }

 private:
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup* jmg_{nullptr};
  const moveit::core::LinkModel* tip_model_{nullptr};
  std::string planning_group_;
  std::string tip_link_;
  double svd_thresh_{0.05};
  double damping_gain_{0.1};
  std::array<double, 6> max_qdot_{};
  std::array<double, 6> max_qddot_{};
  std::array<double, 6> max_qdot_scale_{};
  bool ik_initialized_{false};
  Eigen::VectorXd last_qdot_ik_;
};

// ---- 运行时参数定义 ----
// 该结构包含节点初始化和运行时可配置的参数，通过 ROS 参数服务器加载。
struct RuntimeParams {
  std::string robot_ip{"192.168.1.199"};
  std::string pc_ip{"192.168.1.150"};
  bool mode{true};
  std::string external_control{"external_control.script"};
  std::string output_recipe{"output_recipe.txt"};
  std::string input_recipe{"input_recipe.txt"};
  std::string task_file{"mjktest.task"};
  int sdk_hz{250};
  int pub_hz{100};
  double cmd_timeout{0.05};
  std::string haptic_topic{"/phantom/pose"};
  std::string joint_pub_topic{"/cs66/joint_states"};
  std::string tcp_pub_topic{"/cs66/tcp_state"};
  std::string force_pub_topic{"/phantom/force_feedback"};
  std::string planning_group{"planning_group"};
  std::string tip_link{"ee_link"};
  ELITE::vector6d_t root_joint_pose{};
  std::vector<double> max_qdot_scale{6, 0.05};
};

// 从参数服务器读取运行时配置，允许通过 launch 文件或 rosparam 修改行为。
void loadRuntimeParams(ros::NodeHandle& pnh, RuntimeParams& p) {
  pnh.param("DEFAULT_ROBOT_IP", p.robot_ip, p.robot_ip);
  pnh.param("DEFAULT_PC_IP", p.pc_ip, p.pc_ip);
  pnh.param("MODE", p.mode, p.mode);
  pnh.param("external_control_file_address", p.external_control, p.external_control);
  pnh.param("output_recipe_file_address", p.output_recipe, p.output_recipe);
  pnh.param("input_recipe_file_address", p.input_recipe, p.input_recipe);
  pnh.param("task_file_address", p.task_file, p.task_file);
  pnh.param("robot_sdk_rate", p.sdk_hz, p.sdk_hz);
  pnh.param("robot_pub_rate", p.pub_hz, p.pub_hz);
  pnh.param("cmd_timeout", p.cmd_timeout, p.cmd_timeout);
  pnh.param("haptic_tcp_topic", p.haptic_topic, p.haptic_topic);
  pnh.param("joint_states_pub_topic", p.joint_pub_topic, p.joint_pub_topic);
  pnh.param("tcp_state_pub_topic", p.tcp_pub_topic, p.tcp_pub_topic);
  pnh.param("force_pub_topic", p.force_pub_topic, p.force_pub_topic);
  pnh.param("planning_group", p.planning_group, p.planning_group);
  pnh.param("tip_link", p.tip_link, p.tip_link);
  for (int i = 0; i < 6; ++i) {
    double root_def = 0.0;
    if (i == 1) {
      root_def = -1.18;
    }
    if (i == 2) {
      root_def = -2.44;
    }
    if (i == 3) {
      root_def = -1.1;
    }
    if (i == 4) {
      root_def = 1.57;
    }
    if (i == 5) {
      root_def = -1.57;
    }
    pnh.param("root_joint_" + std::to_string(i + 1), p.root_joint_pose[i], root_def);
    pnh.param("MAX_QDOT_SCALE_" + std::to_string(i + 1), p.max_qdot_scale[i], 0.05);
  }
}

// ---- 机器人的主控制节点 ----
// 负责整合手柄订阅、映射、PID 控制、IK 求解、SDK 输出和状态发布。
class RobotMainNode {
 public:
  explicit RobotMainNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    loadRuntimeParams(pnh_, params_);
    pose_mapper_.load(pnh_);
    controller_.load(pnh_);
    pnh_.param("desired_timeout", desired_timeout_sec_, 0.1);
    controller_.setDesiredTimeout(desired_timeout_sec_);
    ik_.loadIkParams(pnh_);
    std::string ik_err;
    if (!ik_.init(params_.planning_group, params_.tip_link, &ik_err)) {
      ROS_FATAL("MoveIt model load failed: %s", ik_err.c_str());
      throw std::runtime_error(ik_err);
    }

    static constexpr double sdk_max[6] = {5 * kPi / 6, 5 * kPi / 6, kPi, 23 * kPi / 18, 23 * kPi / 18, 23 * kPi / 18};
    for (int i = 0; i < 6; ++i) {
      max_qdot_exec_[i] = sdk_max[i] * params_.max_qdot_scale[i];
    }

    robot_ = std::make_unique<EliteCSRobotSDK>(params_.robot_ip, params_.pc_ip, params_.mode, params_.external_control,
                                                 params_.output_recipe, params_.input_recipe, params_.task_file,
                                                 static_cast<double>(params_.sdk_hz));
    if (!robot_->init() || !robot_->start()) {
      throw std::runtime_error("Robot init/start failed");
    }
    // 机器人连接后先回到预设根位姿，避免在未初始化的姿态下直接执行遥操作。
    moveToRootPose();

    haptic_sub_ = nh_.subscribe<robot_set::TCPState>(params_.haptic_topic, 1, &RobotMainNode::onHaptic, this);
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>(params_.joint_pub_topic, 10);
    tcp_pub_ = nh_.advertise<robot_set::TCPState>(params_.tcp_pub_topic, 10);
    force_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(params_.force_pub_topic, 10);
  }

  ~RobotMainNode() {
    shutdown_ = true;
    if (sdk_thread_.joinable()) {
      sdk_thread_.join();
    }
    if (pub_joint_thread_.joinable()) {
      pub_joint_thread_.join();
    }
    if (pub_tcp_thread_.joinable()) {
      pub_tcp_thread_.join();
    }
    if (pub_force_thread_.joinable()) {
      pub_force_thread_.join();
    }
    if (robot_) {
      robot_->jointSpeed(joint_zero_speed_, 0);
      robot_->disconnect();
    }
  }

  void startThreads() {
    sdk_thread_ = std::thread(&RobotMainNode::sdkLoop, this);
    pub_joint_thread_ = std::thread(&RobotMainNode::publishJoints, this);
    pub_tcp_thread_ = std::thread(&RobotMainNode::publishTcp, this);
    pub_force_thread_ = std::thread(&RobotMainNode::publishForce, this);
  }

 private:
  // 将机械臂移动到预设根位姿。该函数主要在初始化阶段调用，
  // 使机器人从当前状态平稳进入遥操作起始状态。
  void moveToRootPose() {
    double max_joint = 0.0;
    ELITE::vector6d_t current = robot_->getCurrentJoint();
    for (int i = 0; i < 6; ++i) {
      max_joint = std::max(std::abs(params_.root_joint_pose[i] - current[i]), max_joint);
    }
    const double arrive_time = std::max(2.0 * max_joint / max_qdot_exec_[0], 3.0);
    ROS_INFO("Servo move to root pose, estimated time: %.2f s", arrive_time);
    robot_->moveJoint_servo(params_.root_joint_pose, arrive_time);
  }

  // 手柄数据回调：接收期望 TCP 状态并映射到机器人目标空间。
  void onHaptic(const robot_set::TCPState::ConstPtr& msg) {
    robot_set::TCPState mapped;
    pose_mapper_.map(*msg, mapped);
    std::lock_guard<std::mutex> lk(desired_mtx_);
    mapped_desired_ = std::move(mapped);
    desired_valid_ = true;
    t_desired_ = ros::Time::now();
  }

  // SDK 控制循环：周期读取机器人状态，计算控制命令，并发送给底层 SDK。
  void sdkLoop() {
    ros::Rate rate(params_.sdk_hz);
    const double dt = 1.0 / static_cast<double>(params_.sdk_hz);
    while (ros::ok() && !shutdown_) {
      const double now = ros::Time::now().toSec();
      ELITE::vector6d_t joints = robot_->getCurrentJoint();
      ELITE::vector6d_t tcp = robot_->getCurrentTCPPose();
      ELITE::vector6d_t force = robot_->getTCPforce();
      {
        std::lock_guard<std::mutex> lk(state_mtx_);
        joint_cache_ = joints;
        tcp_cache_ = tcp;
        force_cache_ = force;
        updateTcpVelocityEstimateLocked(now);
      }

      Eigen::VectorXd actual6(6);
      Eigen::VectorXd dpose(6);
      Eigen::VectorXd dvel(6);
      for (int i = 0; i < 6; ++i) {
        actual6[i] = tcp[i];
      }
      bool des_ok = false;
      ros::Time t_des_copy;
      {
        std::lock_guard<std::mutex> lk(desired_mtx_);
        des_ok = desired_valid_;
        t_des_copy = t_desired_;
        if (des_ok && mapped_desired_.position.size() >= 6 && mapped_desired_.velocity.size() >= 6) {
          for (int i = 0; i < 6; ++i) {
            dpose[i] = mapped_desired_.position[i];
            dvel[i] = mapped_desired_.velocity[i];
          }
        }
      }
      const double age_des = des_ok ? (now - t_des_copy.toSec()) : 999.0;
      // 如果期望命令有效，则计算笛卡尔速度命令并执行 IK。
      const bool feed_ok = des_ok && age_des < desired_timeout_sec_;
      Eigen::VectorXd v_cmd = controller_.compute(actual6, dpose, dvel, now, feed_ok, age_des);

      Eigen::VectorXd qdot6(6);
      bool ik_ok = false;
      if (feed_ok && v_cmd.norm() > 1e-9) {
        double q_arr[6];
        for (int i = 0; i < 6; ++i) {
          q_arr[i] = joints[i];
        }
        ik_ok = ik_.solve(q_arr, v_cmd, qdot6, dt);
      }

      ELITE::vector6d_t qdot_cmd{};
      bool valid_cmd = false;
      if (ik_ok) {
        for (int i = 0; i < 6; ++i) {
          double qd = qdot6[i];
          qd = clampScalar(qd, -max_qdot_exec_[i], max_qdot_exec_[i]);
          qdot_cmd[i] = qd;
        }
        valid_cmd = true;
        last_cmd_time_.store(now, std::memory_order_release);
      } else if (!feed_ok) {
        ik_.resetIkState();
      }

      const double last_cmd = last_cmd_time_.load(std::memory_order_acquire);
      if (valid_cmd && (now - last_cmd) < params_.cmd_timeout) {
        // 有效命令持续发送；否则发送零速停止机器人。
        robot_->jointSpeed(qdot_cmd, 0);
      } else {
        robot_->jointSpeed(joint_zero_speed_, 0);
      }

      rate.sleep();
    }
  }

  // 基于当前 TCP 位姿和上次位姿估算 TCP 线速度和角速度，用于发布状态信息。
  void updateTcpVelocityEstimateLocked(double now_sec) {
    Eigen::Vector3d p(tcp_cache_[0], tcp_cache_[1], tcp_cache_[2]);
    Eigen::Matrix3d R = eulerXYZToRotForTcp(tcp_cache_[3], tcp_cache_[4], tcp_cache_[5]);
    if (!tcp_vel_init_) {
      tcp_prev_p_ = p;
      tcp_prev_R_ = R;
      tcp_vel_prev_time_ = now_sec;
      tcp_vel_init_ = true;
      tcp_vel_linear_.setZero();
      tcp_vel_angular_.setZero();
      return;
    }
    double dt = now_sec - tcp_vel_prev_time_;
    tcp_vel_prev_time_ = now_sec;
    if (dt < 1e-4 || dt > 0.25) {
      tcp_prev_p_ = p;
      tcp_prev_R_ = R;
      tcp_vel_linear_.setZero();
      tcp_vel_angular_.setZero();
      return;
    }
    tcp_vel_linear_ = (p - tcp_prev_p_) / dt;
    Eigen::Matrix3d dR = tcp_prev_R_.transpose() * R;
    Eigen::AngleAxisd aa(dR);
    tcp_vel_angular_ = aa.axis() * aa.angle() / dt;
    tcp_prev_p_ = p;
    tcp_prev_R_ = R;
  }

  // 发布当前关节状态，便于其他节点订阅和监控。
  void publishJoints() {
    ros::Rate rate(params_.pub_hz);
    sensor_msgs::JointState msg;
    msg.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    msg.position.resize(6);
    while (ros::ok() && !shutdown_) {
      {
        std::lock_guard<std::mutex> lk(state_mtx_);
        for (int i = 0; i < 6; ++i) {
          msg.position[i] = joint_cache_[i];
        }
      }
      msg.header.stamp = ros::Time::now();
      joint_pub_.publish(msg);
      rate.sleep();
    }
  }

  // 发布当前 TCP 位姿与估算的 TCP 速度，用于状态可视化和调试。
  void publishTcp() {
    ros::Rate rate(params_.pub_hz);
    robot_set::TCPState msg;
    msg.position.resize(6);
    msg.velocity.resize(6);
    while (ros::ok() && !shutdown_) {
      {
        std::lock_guard<std::mutex> lk(state_mtx_);
        for (int i = 0; i < 6; ++i) {
          msg.position[i] = tcp_cache_[i];
        }
        for (int i = 0; i < 3; ++i) {
          msg.velocity[i] = tcp_vel_linear_[i];
        }
        for (int i = 0; i < 3; ++i) {
          msg.velocity[i + 3] = tcp_vel_angular_[i];
        }
      }
      msg.header.stamp = ros::Time::now();
      tcp_pub_.publish(msg);
      rate.sleep();
    }
  }

  // 发布力传感器数据，包含机器人端口读到的六维力/力矩。
  void publishForce() {
    ros::Rate rate(params_.pub_hz);
    std_msgs::Float64MultiArray msg;
    msg.data.resize(6);
    while (ros::ok() && !shutdown_) {
      {
        std::lock_guard<std::mutex> lk(state_mtx_);
        for (int i = 0; i < 6; ++i) {
          msg.data[i] = force_cache_[i];
        }
      }
      force_pub_.publish(msg);
      rate.sleep();
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  RuntimeParams params_;
  PoseMapper pose_mapper_;
  PoseErrorController controller_;
  MoveItVelocityIk ik_;
  std::unique_ptr<EliteCSRobotSDK> robot_;

  ros::Subscriber haptic_sub_;
  ros::Publisher joint_pub_;
  ros::Publisher tcp_pub_;
  ros::Publisher force_pub_;

  std::thread sdk_thread_;
  std::thread pub_joint_thread_;
  std::thread pub_tcp_thread_;
  std::thread pub_force_thread_;
  std::atomic<bool> shutdown_{false};

  std::mutex state_mtx_;
  ELITE::vector6d_t joint_cache_{};
  ELITE::vector6d_t tcp_cache_{};
  ELITE::vector6d_t force_cache_{};

  Eigen::Vector3d tcp_vel_linear_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tcp_vel_angular_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tcp_prev_p_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d tcp_prev_R_{Eigen::Matrix3d::Identity()};
  double tcp_vel_prev_time_{0.0};
  bool tcp_vel_init_{false};

  std::mutex desired_mtx_;
  robot_set::TCPState mapped_desired_;
  bool desired_valid_{false};
  ros::Time t_desired_;
  double desired_timeout_sec_{0.1};

  std::array<double, 6> max_qdot_exec_{};
  std::atomic<double> last_cmd_time_{0.0};

  const ELITE::vector6d_t joint_zero_speed_{0, 0, 0, 0, 0, 0};
};

int main(int argc, char** argv) {
  // 程序入口：初始化 ROS 节点，启动异步 spinner，并运行主控制节点。
  ros::init(argc, argv, "robot_main_node");
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  try {
    RobotMainNode node(nh, pnh);
    node.startThreads();
    ros::waitForShutdown();
  } catch (const std::exception& e) {
    ROS_FATAL("%s", e.what());
    return 1;
  }
  return 0;
}
