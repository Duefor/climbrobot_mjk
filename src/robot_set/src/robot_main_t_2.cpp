/**
 * Integrated teleop node: haptic TCP -> workspace mapping -> Cartesian PID ->
 * MoveIt pose IK -> Elite SDK writeservoj (joint position servo).
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
#include <visualization_msgs/Marker.h>

#include <actionlib/client/simple_action_client.h>
#include <realsense_set/ScanEnvironmentAction.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <robot_set/TCPState.h>
#include <robot_sdk_wrapper/robot_sdk.h>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

// 全局常量
constexpr double kEps = 1e-6;
constexpr double kPi = 3.14159265358979323846;

// 将数值限制到指定范围内，常用于速度、角度等限制。
inline double clampScalar(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

static Eigen::Matrix3d rotvecToRot(const Eigen::Vector3d& r) {
  const double theta = r.norm();
  if (theta < 1e-8) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(theta, r / theta).toRotationMatrix();
}

static double unwrapToClosest(double q, double q_ref) {
  const double delta = q - q_ref;
  const double k = std::round(delta / (2.0 * kPi));
  return q - k * 2.0 * kPi;
}

// 将 XYZ 欧拉角转换为 TCP 末端的旋转矩阵。
// 这里采用 ZYX 顺序，即先绕 X 旋转，再绕 Y，最后绕 Z。
inline Eigen::Matrix3d eulerXYZToRotForTcp(double rx, double ry, double rz) {
  Eigen::AngleAxisd Rx(rx, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd Ry(ry, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Rz(rz, Eigen::Vector3d::UnitZ());
  return (Rz * Ry * Rx).toRotationMatrix();
}

// ---- 手柄位姿到机器人工作空间映射 ----
// 该模块将手柄的 TCP 位姿转换为机器人期望的 TCP 位姿。
// 仅对平移部分做线性比例映射，姿态部分根据 CONTROL_MODE 决定是否传递手柄姿态。
// 速度映射已移除，PoseMapper 现在只输出位置。速度输入不再使用。
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
    pnh.param("map_mode", map_mode_, std::string("drift"));
    pnh.param("h_work_radius", h_work_radius_, 0.06);
    pnh.param("h_drift_radius", h_drift_radius_, 0.12);
    pnh.param("drift_gain", drift_gain_, 1.0);
    pnh.param("drift_speed_max", drift_speed_max_, 0.05);
    pnh.param("map_scale", map_scale_override_, 0.0);
    pnh.param("work_box_scale", work_box_scale_, 0.85);

    for (int i = 0; i < 3; ++i) {
      const double denom = std::max(h_max_[i] - h_min_[i], kEps);
      scale_[i] = (r_max_[i] - r_min_[i]) / denom;
    }
    if (map_scale_override_ > 0.0) {
      scale_ = Eigen::Vector3d::Constant(map_scale_override_);
    }

    const Eigen::Vector3d drift_pos_default = (h_max_ - h_init_).cwiseMax(0.0);
    const Eigen::Vector3d drift_neg_default = (h_init_ - h_min_).cwiseMax(0.0);
    for (int i = 0; i < 3; ++i) {
      h_drift_pos_[i] = drift_pos_default[i];
      h_drift_neg_[i] = drift_neg_default[i];
      h_work_pos_[i] = drift_pos_default[i] * work_box_scale_;
      h_work_neg_[i] = drift_neg_default[i] * work_box_scale_;
      if (h_drift_pos_[i] < h_work_pos_[i]) {
        h_drift_pos_[i] = h_work_pos_[i];
      }
      if (h_drift_neg_[i] < h_work_neg_[i]) {
        h_drift_neg_[i] = h_work_neg_[i];
      }
    }

    if (h_drift_radius_ < h_work_radius_) {
      h_drift_radius_ = h_work_radius_;
    }
    h_center_ = h_init_;
    r_center_ = r_init_;
    r_radius_ = scale_.mean() * h_work_radius_;
    last_map_time_ = ros::Time(0.0);
  }

  void map(const robot_set::TCPState& in, robot_set::TCPState& out) {
    if (in.position.size() < 6) {
      return;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    Eigen::Vector3d h_pos(in.position[0], in.position[1], in.position[2]);
    Eigen::Vector3d h_rot(in.position[3], in.position[4], in.position[5]);
    Eigen::Vector3d r_pos;
    const ros::Time now = ros::Time::now();
    double dt = 0.0;
    if (!last_map_time_.isZero()) {
      dt = (now - last_map_time_).toSec();
    }
    last_map_time_ = now;

    if (map_mode_ == "fixed") {
      for (int i = 0; i < 3; ++i) {
        r_pos[i] = mapAxis(h_pos[i], h_init_[i], h_min_[i], h_max_[i], r_init_[i], r_min_[i], r_max_[i]);
      }
      r_radius_ = scale_.mean() * h_work_radius_;
    } else if (map_mode_ == "clutch") {
      const Eigen::Vector3d h_delta = h_pos - h_center_;
      const double dist = h_delta.norm();
      if (dist <= h_work_radius_) {
        r_pos = r_center_ + scale_.cwiseProduct(h_delta);
      } else {
        const Eigen::Vector3d dir = h_delta / std::max(dist, kEps);
        const Eigen::Vector3d h_clamped = h_center_ + dir * h_work_radius_;
        r_pos = r_center_ + scale_.cwiseProduct(h_clamped - h_center_);
        const double over = dist - h_work_radius_;
        r_center_ += scale_.cwiseProduct(dir * over) * drift_gain_;
        h_center_ += dir * over;
      }
      r_radius_ = scale_.mean() * h_work_radius_;
    } else if (map_mode_ == "drift") {
      const Eigen::Vector3d h_delta = h_pos - h_center_;
      Eigen::Vector3d h_clamped = h_center_;
      bool in_work = true;
      for (int i = 0; i < 3; ++i) {
        const double d = h_delta[i];
        if (d > h_work_pos_[i] || d < -h_work_neg_[i]) {
          in_work = false;
        }
        h_clamped[i] = h_center_[i] + clampScalar(d, -h_work_neg_[i], h_work_pos_[i]);
      }
      if (in_work) {
        r_pos = r_center_ + scale_.cwiseProduct(h_delta);
      } else {
        r_pos = r_center_ + scale_.cwiseProduct(h_clamped - h_center_);
        if (dt > 0.0) {
          Eigen::Vector3d drift_vel = Eigen::Vector3d::Zero();
          for (int i = 0; i < 3; ++i) {
            const double d = h_delta[i];
            if (d >= 0.0) {
              const double cap = std::min(d, h_drift_pos_[i]);
              const double excess = std::max(0.0, cap - h_work_pos_[i]);
              if (excess > 0.0) {
                double v = drift_gain_ * excess;
                v = std::min(v, drift_speed_max_);
                drift_vel[i] = v;
              }
            } else {
              const double cap = std::max(d, -h_drift_neg_[i]);
              const double excess = std::max(0.0, -cap - h_work_neg_[i]);
              if (excess > 0.0) {
                double v = drift_gain_ * excess;
                v = std::min(v, drift_speed_max_);
                drift_vel[i] = -v;
              }
            }
          }
          // 表面 safety 联动：被边界夹住时，去掉 drift_vel 沿撞墙法向的分量。
          // safety_block_normal_ 指向工作空间内部（末端被推回、远离表面的方向）。
          // 手柄推向表面 → drift_vel 朝表面 = 与 safety_normal 反向 → 点积<0 → 抑制；
          // 手柄远离表面 → drift_vel 朝 safety_normal 同向 → 点积>0 → 不抑制（允许离开）。
          if (safety_block_active_) {
            const double along = drift_vel.dot(safety_block_normal_);
            if (along < 0.0) {
              drift_vel -= safety_block_normal_ * along;
            }
          }
          r_center_ += drift_vel * dt;
        }
      }
      r_radius_ = scale_.mean() * h_work_radius_;
    }
    out.position.resize(6);
    out.position[0] = r_pos[0];
    out.position[1] = r_pos[1];
    out.position[2] = r_pos[2];
    if (control_mode_ == 0) {
      // 模式0: 固定姿态朝下
      out.position[3] = kPi;
      out.position[4] = 0.0;
      out.position[5] = 0.0;
    } else {
      // 模式1: 手柄姿态直传; 模式2: 手柄姿态作为 fallback，表面感知混合在下游处理
      out.position[3] = h_rot[0];
      out.position[4] = h_rot[1];
      out.position[5] = h_rot[2];
    }
    out.velocity.clear();
    out.header.stamp = now;
  }

  int controlMode() const { return control_mode_; }

  void resetCenters() {
    std::lock_guard<std::mutex> lk(mtx_);
    h_center_ = h_init_;
    r_center_ = r_init_;
    last_map_time_ = ros::Time(0.0);
  }

  void getWorkspace(Eigen::Vector3d& center, double& radius) const {
    std::lock_guard<std::mutex> lk(mtx_);
    center = r_center_;
    radius = r_radius_;
  }

  void getWorkspaceBoxes(Eigen::Vector3d& drift_center, Eigen::Vector3d& drift_size,
                         Eigen::Vector3d& work_center, Eigen::Vector3d& work_size) const {
    std::lock_guard<std::mutex> lk(mtx_);
    const Eigen::Vector3d drift_offset = 0.5 * (h_drift_pos_ - h_drift_neg_);
    const Eigen::Vector3d work_offset = 0.5 * (h_work_pos_ - h_work_neg_);
    drift_center = r_center_ + scale_.cwiseProduct(drift_offset);
    work_center = r_center_ + scale_.cwiseProduct(work_offset);
    drift_size = scale_.cwiseProduct(h_drift_pos_ + h_drift_neg_);
    work_size = scale_.cwiseProduct(h_work_pos_ + h_work_neg_);
  }

  // sdkLoop 检测到末端被表面边界夹住时调用：传入安全法向（指向工作空间内部、
  // 即 safety 钳制把末端推回的方向）与激活标志。map() 的 drift 分支会据此把
  // drift_vel 沿该法向的分量去掉，只保留切向漂移，防止 r_center_ 朝撞墙方向累积。
  // active=false 时 normal 被忽略，drift 完全恢复（出区域或无表面数据）。
  void setSafetyBlock(const Eigen::Vector3d& normal, bool active) {
    std::lock_guard<std::mutex> lk(mtx_);
    safety_block_normal_ = active ? normal : Eigen::Vector3d::Zero();
    safety_block_active_ = active;
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
    static const double d[3] = {-0.030672866821289058, -0.21, -0.1};
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
    static const double d[3] = {0.28, -0.56, 0.12};
    // static const double d[3] = {0.28, -0.56, 0.22};
    return d[i];
  }

  static double mapAxis(double h, double h0, double hmin, double hmax, double r0, double rmin, double rmax) {
    const double dh = h - h0;
    const double gain_pos = (rmax - r0) / std::max(hmax - h0, kEps);
    const double gain_neg = (r0 - rmin) / std::max(h0 - hmin, kEps);
    const double gain = (dh >= 0.0) ? gain_pos : gain_neg;
    return clampScalar(r0 + gain * dh, rmin, rmax);
  }

  Eigen::Vector3d h_init_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d h_max_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d h_min_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d r_init_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d r_max_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d r_min_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d h_center_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d r_center_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d scale_{Eigen::Vector3d::Ones()};
  double h_work_radius_{0.06};
  double h_drift_radius_{0.12};
  Eigen::Vector3d h_work_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d h_work_neg_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d h_drift_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d h_drift_neg_{Eigen::Vector3d::Zero()};
  double r_radius_{0.0};
  double drift_gain_{1.0};
  double drift_speed_max_{0.05};
  double map_scale_override_{0.0};
  double work_box_scale_{0.85};
  std::string map_mode_{"drift"};
  int control_mode_{0};
  // 表面 safety 联动：sdkLoop 检测到末端被边界夹住时，把安全法向写入此，
  // map() 累积 drift 时去掉该法向分量，避免 r_center_ 朝撞墙方向偷跑 → 松手跳变。
  Eigen::Vector3d safety_block_normal_{Eigen::Vector3d::Zero()};
  bool safety_block_active_{false};
  mutable std::mutex mtx_;
  ros::Time last_map_time_;
};

// ---- 笛卡尔空间位姿 PID 控制器 ----
// 基于实际 TCP 与期望 TCP 计算跟踪误差，输出修正后的笛卡尔目标位姿。
// - 位置误差使用标准 PID，输出位置修正量叠加到期望位姿
// - 旋转误差使用纯比例控制，输出旋转修正叠加到期望姿态
// - beta 用于在大误差时逐步引入控制，避免初始突变
class PoseErrorController {
 public:
  void load(ros::NodeHandle& pnh) {
    double kp_lin_n = 0.1, ki_lin_n = 0.002, kd_lin_n = 0.03;
    double kp_rot_n = 0.002;
    pnh.param("KP_LIN_N", kp_lin_n, kp_lin_n);
    pnh.param("KI_LIN_N", ki_lin_n, ki_lin_n);
    pnh.param("KD_LIN_N", kd_lin_n, kd_lin_n);
    pnh.param("KP_ROT_N", kp_rot_n, kp_rot_n);
    pnh.param("EQUAL1", equal1_, 0.02);
    pnh.param("EQUAL2", equal2_, 0.43);
    pnh.param("integral_limit", integral_limit_, 2.0);
    pnh.param("beta_tau", beta_tau_, 0.3);
    kp_lin_ = kp_lin_n * Eigen::Matrix3d::Identity();
    ki_lin_ = ki_lin_n * Eigen::Matrix3d::Identity();
    kd_lin_ = kd_lin_n * Eigen::Matrix3d::Identity();
    kp_rot_ = kp_rot_n * Eigen::Matrix3d::Identity();
    reset();
  }

  void reset() {
    error_prev_.setZero();
    error_integral_.setZero();
    beta_ = 0.0;
    r_d_prev_ = Eigen::Matrix3d::Identity();
    r_d_init_ = false;
    last_ctl_time_valid_ = false;
  }

  // 返回修正后的笛卡尔目标位姿 [x,y,z,rx,ry,rz]（在 base_link 下）
  Eigen::VectorXd compute(const Eigen::VectorXd& actual6, const Eigen::VectorXd& desired_pose,
                          double now_sec, bool desired_ready,
                          double desired_age_sec) {
    Eigen::VectorXd cmd_pose = desired_pose;
    if (!last_ctl_time_valid_) {
      last_ctl_time_ = now_sec;
      last_ctl_time_valid_ = true;
      return cmd_pose;
    }
    double dt = now_sec - last_ctl_time_;
    last_ctl_time_ = now_sec;
    if (dt <= 1e-6 || dt > 0.05) {
      return cmd_pose;
    }

    if (!desired_ready || desired_age_sec > desired_timeout_) {
      resetPartial();
      return cmd_pose;
    }

    // ---- SE(3) 误差计算 ----
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

    // ---- beta 软启动 ----
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

    // ---- 积分（仅平移） ----
    error_integral_.head<3>() += xi.head<3>() * dt;
    error_integral_.tail<3>().setZero();
    for (int i = 0; i < 3; ++i) {
      error_integral_[i] = clampScalar(error_integral_[i], -integral_limit_, integral_limit_);
    }

    // ---- 微分（仅平移） ----
    Eigen::VectorXd error_derivative = Eigen::VectorXd::Zero(6);
    error_derivative.head<3>() = (xi.head<3>() - error_prev_.head<3>()) / dt;
    error_prev_ = xi;

    // ---- 位姿修正量 = beta * (Kp*err + Ki*integral + Kd*deriv) ----
    Eigen::VectorXd correction = Eigen::VectorXd::Zero(6);
    correction.head<3>() = beta_ * (kp_lin_ * xi.head<3>() +
                                    ki_lin_ * error_integral_.head<3>() +
                                    kd_lin_ * error_derivative.head<3>());
    correction.tail<3>() = beta_ * (kp_rot_ * xi.tail<3>());

    // ---- 应用于期望位姿 ----
    cmd_pose.head<3>() = desired_pose.head<3>() + correction.head<3>();

    // 旋转修正：将 angle-axis 修正量叠加到期望旋转上
    Eigen::Matrix3d R_corr = rotvecToRot(correction.tail<3>());
    Eigen::Matrix3d R_new = T_d.block<3, 3>(0, 0) * R_corr;
    Eigen::AngleAxisd aa_new(R_new);
    cmd_pose.tail<3>() = aa_new.axis() * aa_new.angle();
    // 更新期望旋转历史（用于计算虚拟角速度，当前保留用于 resetPartial）
    r_d_prev_ = T_d.block<3, 3>(0, 0);

    return cmd_pose;
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
  double equal1_{0.02};
  double equal2_{0.43};
  double integral_limit_{2.0};
  double beta_tau_{0.3};
  double desired_timeout_{0.1};

  Eigen::VectorXd error_prev_{Eigen::VectorXd::Zero(6)};
  Eigen::VectorXd error_integral_{Eigen::VectorXd::Zero(6)};
  double beta_{0.0};
  Eigen::Matrix3d r_d_prev_{Eigen::Matrix3d::Identity()};
  bool r_d_init_{false};
  double last_ctl_time_{0.0};
  bool last_ctl_time_valid_{false};
};

// ---- MoveIt 位姿 IK ----
// 将 6D 笛卡尔目标位姿转换为关节角度。仿真和实机共用。
class MoveItPoseIk {
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
    if (!robot_model_->getLinkModel(tip_link_)) {
      if (err) {
        *err = "Unknown tip link: " + tip_link_;
      }
      return false;
    }
    joint_names_ = jmg_->getVariableNames();
    if (joint_names_.size() < 6) {
      joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    }
    return true;
  }

  void loadIkParams(ros::NodeHandle& pnh) {
    pnh.param("ik_timeout", ik_timeout_, 0.01);
    pnh.param("ik_attempts", ik_attempts_, 3);
  }

  // 实机用：输入目标 TCP 位姿 [x,y,z,rx,ry,rz] 和当前关节角，输出目标关节角
  bool solve(const Eigen::VectorXd& target_pose, const double current_joints[6],
             ELITE::vector6d_t& q_out) {
    if (!robot_model_ || !jmg_) {
      return false;
    }
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.translation() = target_pose.head<3>();
    target.linear() = rotvecToRot(target_pose.tail<3>());

    std::vector<double> seed(6);
    for (int i = 0; i < 6; ++i) {
      seed[i] = current_joints[i];
    }

    std::vector<double> sol;
    const bool ok = solve(target, seed, sol, ik_timeout_);
    if (!ok || sol.size() < 6) {
      return false;
    }

    // 选最接近当前关节角的解（unwrap 后）
    for (int i = 0; i < 6; ++i) {
      sol[i] = unwrapToClosest(sol[i], current_joints[i]);
    }
    for (int i = 0; i < 6; ++i) {
      q_out[i] = sol[i];
    }
    return true;
  }

  bool solve(const Eigen::Isometry3d& target, const std::vector<double>& seed,
             std::vector<double>& solution, double timeout) const {
    if (!robot_model_ || !jmg_) {
      return false;
    }
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    if (!seed.empty()) {
      state.setJointGroupPositions(jmg_, seed);
    }
    state.update();
    if (!state.setFromIK(jmg_, target, tip_link_, timeout)) {
      return false;
    }
    state.copyJointGroupPositions(jmg_, solution);
    return solution.size() >= 6;
  }

  const std::vector<std::string>& jointNames() const { return joint_names_; }

 private:
  static Eigen::Matrix3d rotvecToRot(const Eigen::Vector3d& r) {
    const double theta = r.norm();
    if (theta < 1e-8) {
      return Eigen::Matrix3d::Identity();
    }
    return Eigen::AngleAxisd(theta, r / theta).toRotationMatrix();
  }

  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup* jmg_{nullptr};
  std::string planning_group_;
  std::string tip_link_;
  std::vector<std::string> joint_names_;
  double ik_timeout_{0.01};
  int ik_attempts_{3};
};

// ---- 表面采样点处理器 ----
// 专门处理 scan_environment 返回的采样点数据：
// 存储、最近邻查询。线程安全（actionlib 回调写入，sdkLoop 读取）。
class SurfaceSampleProcessor {
 public:
  void setSamples(const std::vector<geometry_msgs::Pose>& samples) {
    std::lock_guard<std::mutex> lk(mtx_);
    samples_ = samples;
    has_data_ = !samples.empty();
    if (samples_.empty()) {
      bbox_valid_ = false;
      return;
    }
    // 一次性 XY 包围盒，用于 isInsideRegion 的确定性区域判定。
    double xmin = std::numeric_limits<double>::infinity();
    double xmax = -std::numeric_limits<double>::infinity();
    double ymin = std::numeric_limits<double>::infinity();
    double ymax = -std::numeric_limits<double>::infinity();
    for (const auto& s : samples_) {
      xmin = std::min(xmin, s.position.x);
      xmax = std::max(xmax, s.position.x);
      ymin = std::min(ymin, s.position.y);
      ymax = std::max(ymax, s.position.y);
    }
    xmin_ = xmin; xmax_ = xmax; ymin_ = ymin; ymax_ = ymax;
    bbox_valid_ = true;
  }

  bool hasData() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return has_data_;
  }

  size_t sampleCount() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return samples_.size();
  }

  // 确定性区域归属判定：actual_pos 的 XY 是否落在采样点云覆盖的矩形内（含边缘容差）。
  // 不依赖 blend 球内有没有点，边界干净、无抖动。
  bool isInsideRegion(const Eigen::Vector3d& pos, double edge_tol) const {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!bbox_valid_) return false;
    return pos.x() >= xmin_ - edge_tol && pos.x() <= xmax_ + edge_tol
        && pos.y() >= ymin_ - edge_tol && pos.y() <= ymax_ + edge_tol;
  }

  // 距离加权法向量平均。在 blend_radius 内找所有采样点，高斯加权平均法向量。
  // 避免了单点最近邻的姿态跳变问题。
  // 返回 true 表示找到至少一个在范围内的点。
  // out_normal:        加权平均后的单位法向量
  // out_point:         加权平均后的表面参考点位置（fallback 用）
  // out_dist:          最近采样点距离
  // out_nearest_pt:    最近采样点的位置（曲面/高低差首选参考）
  // out_nearest_normal:最近采样点的法向量
  bool computeBlendedNormal(const Eigen::Vector3d& tcp_pos,
                            double blend_radius,
                            Eigen::Vector3d& out_normal,
                            Eigen::Vector3d& out_point,
                            double& out_dist,
                            Eigen::Vector3d& out_nearest_pt,
                            Eigen::Vector3d& out_nearest_normal) const {
    std::lock_guard<std::mutex> lk(mtx_);
    if (samples_.empty()) return false;

    const double sigma = blend_radius / 3.0;           // 3σ 半径，边界处权重≈0.01
    const double two_sigma2 = 2.0 * sigma * sigma;
    Eigen::Vector3d weighted_normal = Eigen::Vector3d::Zero();
    Eigen::Vector3d weighted_point = Eigen::Vector3d::Zero();
    double total_weight = 0.0;
    out_dist = std::numeric_limits<double>::max();

    for (const auto& s : samples_) {
      Eigen::Vector3d sp(s.position.x, s.position.y, s.position.z);
      const double d = (sp - tcp_pos).norm();
      // 从四元数提取法向量（Z轴 = 表面法向量）
      Eigen::Quaterniond q(s.orientation.w, s.orientation.x,
                           s.orientation.y, s.orientation.z);
      Eigen::Vector3d normal = q.toRotationMatrix().col(2);
      if (d < out_dist) {
        out_dist = d;
        out_nearest_pt = sp;
        out_nearest_normal = normal;
      }
      if (d > blend_radius) continue;
      const double w = std::exp(-d * d / two_sigma2);
      weighted_normal += w * normal;
      weighted_point += w * sp;
      total_weight += w;
    }

    // out_nearest_pt / out_nearest_normal 只要遍历过任何点就有效，但 blended 信息需要权重。
    out_normal = (total_weight > 1e-9)
        ? (weighted_normal / total_weight).normalized()
        : out_nearest_normal;   // 球内无点：法向退化为最近邻法向
    out_point = (total_weight > 1e-9)
        ? (weighted_point / total_weight)
        : out_nearest_pt;       // 球内无点：点退化为最近邻点
    return true;
  }

 private:
  mutable std::mutex mtx_;
  std::vector<geometry_msgs::Pose> samples_;
  bool has_data_{false};
  double xmin_{0.0}, xmax_{0.0}, ymin_{0.0}, ymax_{0.0};
  bool bbox_valid_{false};
};

// ---- 运行时参数定义 ----
// 该结构包含节点初始化和运行时可配置的参数，通过 ROS 参数服务器加载。
struct RuntimeParams {
  bool use_sim{true};
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
  std::string sim_joint_pub_topic{"/joint_states"};
  std::string tcp_pub_topic{"/cs66/tcp_state"};
  std::string force_pub_topic{"/phantom/force_feedback"};
  std::string planning_group{"planning_group"};
  std::string tip_link{"ee_link"};
  double sim_rate{60.0};
  double sim_ik_timeout{0.01};
  bool workspace_viz_enable{false};
  double workspace_viz_rate{20.0};
  std::string workspace_viz_frame{"base_link"};
  std::string workspace_viz_topic{"/workspace_window"};
  double work_box_scale{0.85};
  ELITE::vector6d_t root_joint_pose{};
  double ik_timeout{0.01};
  int ik_attempts{3};
  double surface_approach_max_dist{0.15};
  double surface_approach_min_dist{0.03};
  double surface_blend_radius{0.10};   // 法向量加权平均的邻域半径
  double surface_lock_delay{0.5};
  double surface_safety_margin{0.2};  // [调试] 末端最小安全距离
  double surface_edge_tol{0.02};            // 区域判定 XY 边缘容差(m)，避免边界抖动
  double surface_out_region_decay{0.3};     // 出区域后缓存保留时长(s)，期间仍约束、超时放手
  double surface_nearest_trust_dist{0.05};  // 最近邻单点法向信任半径(m)
  // drift 漂移区力反馈：机械臂 TCP 超出 work_radius 时，沿指向 r_center_ 方向
  // 施加弹簧力到手柄，让操作者主观感受到工作空间的漂移边界。
  bool   drift_force_enable{true};
  double drift_force_k{40.0};     // 弹簧刚度 (N/m)：超出 work_radius 每米产生的力
  double drift_force_max{2.0};    // 力上限 (N)，避免过大反馈伤设备
};

// 从参数服务器读取运行时配置，允许通过 launch 文件或 rosparam 修改行为。
void loadRuntimeParams(ros::NodeHandle& pnh, RuntimeParams& p) {
  pnh.param("use_sim", p.use_sim, p.use_sim);
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
  pnh.param("sim_joint_states_topic", p.sim_joint_pub_topic, p.sim_joint_pub_topic);
  pnh.param("tcp_state_pub_topic", p.tcp_pub_topic, p.tcp_pub_topic);
  pnh.param("force_pub_topic", p.force_pub_topic, p.force_pub_topic);
  pnh.param("planning_group", p.planning_group, p.planning_group);
  pnh.param("tip_link", p.tip_link, p.tip_link);
  pnh.param("sim_rate", p.sim_rate, p.sim_rate);
  pnh.param("sim_ik_timeout", p.sim_ik_timeout, p.sim_ik_timeout);
  pnh.param("workspace_viz_enable", p.workspace_viz_enable, p.workspace_viz_enable);
  pnh.param("workspace_viz_rate", p.workspace_viz_rate, p.workspace_viz_rate);
  pnh.param("workspace_viz_frame", p.workspace_viz_frame, p.workspace_viz_frame);
  pnh.param("workspace_viz_topic", p.workspace_viz_topic, p.workspace_viz_topic);
  pnh.param("work_box_scale", p.work_box_scale, p.work_box_scale);
  pnh.param("ik_timeout", p.ik_timeout, p.ik_timeout);
  pnh.param("ik_attempts", p.ik_attempts, p.ik_attempts);
  pnh.param("surface_approach_max_dist", p.surface_approach_max_dist, 0.12);
  pnh.param("surface_approach_min_dist", p.surface_approach_min_dist, 0.1);
  pnh.param("surface_blend_radius", p.surface_blend_radius, 0.10);
  pnh.param("surface_lock_delay", p.surface_lock_delay, 0.5);
  pnh.param("surface_safety_margin", p.surface_safety_margin, 0.05);
  pnh.param("surface_edge_tol", p.surface_edge_tol, 0.02);
  pnh.param("surface_out_region_decay", p.surface_out_region_decay, 0.3);
  pnh.param("surface_nearest_trust_dist", p.surface_nearest_trust_dist, 0.05);
  pnh.param("drift_force_enable", p.drift_force_enable, true);
  pnh.param("drift_force_k", p.drift_force_k, 40.0);
  pnh.param("drift_force_max", p.drift_force_max, 3.0);
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

    // IK 初始化（仿真和实机共用 MoveItPoseIk）
    std::string ik_err;
    if (!pose_ik_.init(params_.planning_group, params_.tip_link, &ik_err)) {
      ROS_FATAL("MoveIt pose IK init failed: %s", ik_err.c_str());
      throw std::runtime_error(ik_err);
    }
    pose_ik_.loadIkParams(pnh_);
    sim_joint_names_ = pose_ik_.jointNames();

    if (params_.use_sim) {
      for (int i = 0; i < 6; ++i) {
        sim_seed_[i] = params_.root_joint_pose[i];
      }
      joint_pub_ = nh_.advertise<sensor_msgs::JointState>(params_.sim_joint_pub_topic, 10);
    } else {
      robot_ = std::make_unique<EliteCSRobotSDK>(params_.robot_ip, params_.pc_ip, params_.mode, params_.external_control,
                                                 params_.output_recipe, params_.input_recipe, params_.task_file,
                                                 static_cast<double>(params_.sdk_hz));
      if (!robot_->init() || !robot_->start()) {
        throw std::runtime_error("Robot init/start failed");
      }
      moveToRootPose();

      joint_pub_ = nh_.advertise<sensor_msgs::JointState>(params_.joint_pub_topic, 10);
      tcp_pub_ = nh_.advertise<robot_set::TCPState>(params_.tcp_pub_topic, 10);
      force_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(params_.force_pub_topic, 10);
    }

    // 工作空间/漂移空间可视化：仿真与实机共用，不受 use_sim 门控。
    if (params_.workspace_viz_enable) {
      workspace_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(params_.workspace_viz_topic, 1, true);
    }

    haptic_sub_ = nh_.subscribe<robot_set::TCPState>(params_.haptic_topic, 1, &RobotMainNode::onHaptic, this);

    initSurfaceAware();
  }

  ~RobotMainNode() {
    shutdown_ = true;
    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }
    if (sim_thread_.joinable()) {
      sim_thread_.join();
    }
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
      robot_->stopMove();
      robot_->disconnect();
    }
  }

  void startThreads() {
    if (params_.use_sim) {
      sim_thread_ = std::thread(&RobotMainNode::simLoop, this);
      return;
    }
    sdk_thread_ = std::thread(&RobotMainNode::sdkLoop, this);
    pub_joint_thread_ = std::thread(&RobotMainNode::publishJoints, this);
    pub_tcp_thread_ = std::thread(&RobotMainNode::publishTcp, this);
    pub_force_thread_ = std::thread(&RobotMainNode::publishForce, this);
  }

 private:
  // ---- 表面感知：键盘监听线程 ----
  // 在独立线程中监听 stdin，检测 Enter 键触发扫描。
  void keyboardLoop() {
    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    ROS_INFO("按 Enter 键触发环境扫描...");
    while (!shutdown_.load()) {
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(STDIN_FILENO, &fds);
      struct timeval tv = {0, 100000};  // 100ms 超时
      if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0) {
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0 && (c == '\r' || c == '\n')) {
          scan_requested_.store(true);
          ROS_INFO("收到扫描触发!");
        }
      }
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
  }

  // ---- 表面感知：初始化 ----
  void initSurfaceAware() {
    scan_client_ = std::make_unique<
        actionlib::SimpleActionClient<realsense_set::ScanEnvironmentAction>>(
        "scan_environment", true);
    keyboard_thread_ = std::thread(&RobotMainNode::keyboardLoop, this);
    ROS_INFO("表面感知模块已初始化");
  }

  // ---- 表面感知：发送扫描请求（异步） ----
  void triggerScan() {
    if (!scan_client_) return;
    if (!scan_client_->waitForServer(ros::Duration(2.0))) {
      ROS_WARN("scan_environment 服务不可用");
      scan_done_.store(true);  // 标记完成以退出扫描状态
      return;
    }
    realsense_set::ScanEnvironmentGoal goal;
    goal.grid_rows = 0;     // 0 = 使用服务端默认 (40)
    goal.grid_cols = 0;     // 0 = 使用服务端默认 (30)
    goal.window_size = 0;   // 0 = 使用服务端默认 (7)
    goal.max_depth = 0.0;   // 0.0 = 使用服务端默认 (3.0m)
    scan_client_->sendGoal(goal,
        boost::bind(&RobotMainNode::onScanDone, this, _1, _2));
    ROS_INFO("扫描请求已发送...");
  }

  void onScanDone(
      const actionlib::SimpleClientGoalState& state,
      const realsense_set::ScanEnvironmentResultConstPtr& result) {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED && result) {
      surface_processor_.setSamples(result->samples);
      ROS_INFO("扫描完成: %zu 个采样点已缓存", result->samples.size());
    } else {
      ROS_WARN("扫描失败: %s", state.toString().c_str());
    }
    scan_done_.store(true);
  }

  // ---- 表面感知：法向量 → 姿态四元数 ----
  // 与 scan_environment_server 的 normal_to_quaternion 一致：
  // Z=法向量, X=[1,0,0]投影到法平面, Y=Z×X
  static Eigen::Quaterniond normalToQuaternion(const Eigen::Vector3d& normal) {
    Eigen::Vector3d Z = normal.normalized();
    Eigen::Vector3d X(1.0, 0.0, 0.0);
    X = X - Z * X.dot(Z);  // Gram-Schmidt 投影
    if (X.norm() < 1e-6) {
      X = Eigen::Vector3d(0.0, 1.0, 0.0);
      X = X - Z * X.dot(Z);
    }
    X.normalize();
    Eigen::Vector3d Y = Z.cross(X);
    Y.normalize();
    Eigen::Matrix3d R;
    R.col(0) = X;
    R.col(1) = Y;
    R.col(2) = Z;
    return Eigen::Quaterniond(R);
  }

  // ---- 表面感知：姿态混合计算 ----
  // 阶段1: 邻域高斯加权平均法向量（避免单点跳变）
  // 阶段2: 距离线性过渡 + slerp 混合手柄姿态和表面对齐姿态
  bool computeSurfaceOrientation(
      const Eigen::Vector3d& actual_pos,
      const Eigen::Vector3d& haptic_rotvec,
      Eigen::Vector3d& out_rotvec) {
    out_rotvec = haptic_rotvec;  // 默认：不修改
    if (!surface_processor_.hasData()) return false;

    // 阶段1: 邻域加权平均法向量
    Eigen::Vector3d blended_normal, dummy_pt, dummy_nearest_pt, dummy_nearest_normal;
    double nearest_dist = 0.0;
    if (!surface_processor_.computeBlendedNormal(actual_pos,
          params_.surface_blend_radius, blended_normal, dummy_pt, nearest_dist,
          dummy_nearest_pt, dummy_nearest_normal))
      return false;

    // 阶段2: 距离决定 alpha（用最近点距离判断远近）
    double alpha;
    if (nearest_dist <= params_.surface_approach_min_dist) {
      alpha = 1.0;
    } else if (nearest_dist >= params_.surface_approach_max_dist) {
      alpha = 0.0;
    } else {
      alpha = 1.0 - (nearest_dist - params_.surface_approach_min_dist) /
                    (params_.surface_approach_max_dist - params_.surface_approach_min_dist);
    }
    if (alpha < 1e-6) return false;  // 太远，不干预

    // 法向量 → 完整姿态四元数
    Eigen::Quaterniond q_surface = normalToQuaternion(blended_normal);

    // 手柄旋转向量 → 四元数
    double theta = haptic_rotvec.norm();
    Eigen::Quaterniond q_haptic = (theta < 1e-8)
        ? Eigen::Quaterniond::Identity()
        : Eigen::Quaterniond(Eigen::AngleAxisd(theta, haptic_rotvec / theta));

    // slerp 球面线性插值
    Eigen::Quaterniond q_blended = q_haptic.slerp(alpha, q_surface);

    // 转回旋转向量
    Eigen::AngleAxisd aa(q_blended);
    out_rotvec = aa.axis() * aa.angle();
    return true;
  }

  // 将机械臂移动到预设根位姿。该函数主要在初始化阶段调用，
  // 使机器人从当前状态平稳进入遥操作起始状态。
  void moveToRootPose() {
    double max_joint = 0.0;
    ELITE::vector6d_t current = robot_->getCurrentJoint();
    for (int i = 0; i < 6; ++i) {
      max_joint = std::max(std::abs(params_.root_joint_pose[i] - current[i]), max_joint);
    }
    const double arrive_time = std::max(2.0 * max_joint / 0.2, 3.0);  // 0.2 rad/s ≈ 基准速度
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

  // SDK 控制循环：周期读取机器人状态，PID 修正目标位姿，位置 IK → writeservoj。
  void sdkLoop() {
    ros::Rate rate(params_.sdk_hz);
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
      for (int i = 0; i < 6; ++i) {
        actual6[i] = tcp[i];
      }

      // ---- 读取期望位姿（仅位置，不使用速度） ----
      Eigen::VectorXd dpose(6);
      bool des_ok = false;
      ros::Time t_des_copy;
      {
        std::lock_guard<std::mutex> lk(desired_mtx_);
        des_ok = desired_valid_;
        t_des_copy = t_desired_;
        if (des_ok && mapped_desired_.position.size() >= 6) {
          for (int i = 0; i < 6; ++i) {
            dpose[i] = mapped_desired_.position[i];
          }
        }
      }
      const double age_des = des_ok ? (now - t_des_copy.toSec()) : 999.0;
      const bool feed_ok = des_ok && age_des < desired_timeout_sec_;
      // 手柄断连时工作空间保持当前状态，不重置

      // ====== 表面扫描状态机 ======
      // Enter 键触发后：LOCKING（等待稳定）→ SCANNING（保持锁定+等待扫描结果）→ NORMAL
      if (scan_requested_.load() && scan_state_ == ScanState::NORMAL) {
        scan_state_ = ScanState::LOCKING;
        lock_start_time_ = ros::Time::now();
        for (int i = 0; i < 6; ++i) lock_joints_[i] = joints[i];
        ROS_INFO("表面扫描: 锁定机械臂 %.1fs...", params_.surface_lock_delay);
      }
      if (scan_state_ == ScanState::LOCKING) {
        robot_->writeservoj(lock_joints_, 0);
        if ((ros::Time::now() - lock_start_time_).toSec() >= params_.surface_lock_delay) {
          scan_done_.store(false);
          triggerScan();
          scan_state_ = ScanState::SCANNING;
          ROS_INFO("表面扫描: 正在扫描...");
        }
        rate.sleep();
        continue;
      }
      if (scan_state_ == ScanState::SCANNING) {
        robot_->writeservoj(lock_joints_, 0);
        if (scan_done_.load()) {
          scan_state_ = ScanState::NORMAL;
          scan_requested_.store(false);
          ROS_INFO("表面扫描: 完成，共 %zu 个采样点", surface_processor_.sampleCount());
        }
        rate.sleep();
        continue;
      }

      // ---- 表面感知：姿态混合（仅 CONTROL_MODE=2） ----
      if (feed_ok && pose_mapper_.controlMode() == 2 && surface_processor_.hasData()) {
        Eigen::Vector3d actual_pos = actual6.head<3>();
        Eigen::Vector3d haptic_rotvec = dpose.tail<3>();
        Eigen::Vector3d surface_rotvec;
        if (computeSurfaceOrientation(actual_pos, haptic_rotvec, surface_rotvec)) {
          dpose[3] = surface_rotvec[0];
          dpose[4] = surface_rotvec[1];
          dpose[5] = surface_rotvec[2];
        }
      }

      // ---- PID 控制器：输出修正后的目标笛卡尔位姿 ----
      Eigen::VectorXd cmd_pose = controller_.compute(actual6, dpose, now, feed_ok, age_des);

      // ---- [调试] 安全距离硬边界：clamp cmd_pose 位置（PID 后，IK 前） ----
      // 约束模型（区域内外干净切分）：
      //   表面点云 XY 覆盖的矩形 = 受约束区域
      //   区域内：法向距表面 ≥ surface_safety_margin（不穿透），切向（沿表面 xy）完全自由
      //   区域外：完全自由操控，不施加任何约束
      // 两步解耦：
      //   A. isInsideRegion(actual_pos) 确定性判定是否在区域内（不靠 blend 球里有没有点）
      //   B. 仅在区域内，用最近邻单点法向（曲面/高低差更准）或 blend（远距离 fallback）
      //      做法向正交投影钳制，切向分量完全保留
      if (feed_ok && pose_mapper_.controlMode() == 2 && surface_processor_.hasData()) {
        const Eigen::Vector3d actual_pos = actual6.head<3>();

        // 步骤A：干净的区域归属判定
        if (surface_processor_.isInsideRegion(actual_pos, params_.surface_edge_tol)) {
          // 步骤B：区域内，算表面参考
          Eigen::Vector3d blended_normal, blended_pt, nearest_pt, nearest_normal;
          double nearest_dist;
          if (surface_processor_.computeBlendedNormal(actual_pos,
                params_.surface_blend_radius, blended_normal, blended_pt, nearest_dist,
                nearest_pt, nearest_normal)) {
            // 法向参考：最近邻足够近时优先用单点（曲面/高低差更准），否则 blend 平滑
            Eigen::Vector3d ref_normal = (nearest_dist < params_.surface_nearest_trust_dist)
                ? nearest_normal : blended_normal;
            Eigen::Vector3d ref_pt = (nearest_dist < params_.surface_nearest_trust_dist)
                ? nearest_pt : blended_pt;
            // 服务端法向指离相机（指向工作空间外侧），翻转后指向工作空间内部（机械臂侧），
            // 也就是末端应当退让、保持安全距离的方向。不翻转会让 clamp 变成猛冲。
            ref_normal = -ref_normal;
            // 法向方向连续性保护：与上次夹角>90°(dot<0)说明"翻面"，沿用上次方向，
            // 防止曲面/台阶边缘法向跳变导致 clamp 方向突然反转。
            if (safety_ref_valid_ && ref_normal.dot(safety_normal_) < 0.0) {
              ref_normal = -ref_normal;
            }
            safety_normal_      = ref_normal;   // 已翻转，指向工作空间内部
            safety_surface_pt_  = ref_pt;
            safety_ref_time_    = ros::Time::now();
            safety_ref_valid_   = true;
          }
          // in-region 内若球内无点，保留上一周期参考（decay 内仍生效），不施加无效更新
        } else {
          // 区域外：带时间衰减地放手。短时越界（边缘抖动一两个周期）保留参考避免冲击，
          // 持续越界（超过 decay）彻底清空，机械臂完全自由
          if (safety_ref_valid_ &&
              (ros::Time::now() - safety_ref_time_).toSec() > params_.surface_out_region_decay) {
            safety_ref_valid_ = false;
          }
        }

        // 步骤B 执行：仅在有有效参考时，沿法向正交投影钳制（切向完全保留）
        if (safety_ref_valid_) {
          const Eigen::Vector3d cmd_pos = cmd_pose.head<3>();
          const double signed_dist = (cmd_pos - safety_surface_pt_).dot(safety_normal_);
          if (signed_dist < params_.surface_safety_margin) {
            cmd_pose.head<3>() += safety_normal_ * (params_.surface_safety_margin - signed_dist);
          }
        }

        // drift 抑制联动：把"本周期是否被边界夹住"回传给 PoseMapper。
        // 钳制段若 signed_dist<margin 视为夹住 → PoseMapper 下一周期去掉 drift 沿
        // safety_normal_ 反向（撞墙方向）的分量；切向漂移保留。
        // 无有效参考时显式清空，确保出区域后抑制及时撤销。
        if (safety_ref_valid_) {
          const double signed_dist_after =
              (cmd_pose.head<3>() - safety_surface_pt_).dot(safety_normal_);
          pose_mapper_.setSafetyBlock(
              safety_normal_,
              signed_dist_after < params_.surface_safety_margin);
        } else {
          pose_mapper_.setSafetyBlock(Eigen::Vector3d::Zero(), false);
        }
      }

      // ---- 位置 IK ----
      ELITE::vector6d_t q_cmd{};
      bool ik_ok = false;
      if (feed_ok) {
        ik_ok = pose_ik_.solve(cmd_pose, joints.data(), q_cmd);
      }

      // ---- 伺服写入 ----
      if (ik_ok) {
        // IK 成功：将目标关节角写入伺服
        robot_->writeservoj(q_cmd, 0);
        last_cmd_time_.store(now, std::memory_order_release);
      } else if (feed_ok) {
        // 有手柄信号但 IK 失败：原地保持，超时后发送当前关节角
        const double last_cmd = last_cmd_time_.load(std::memory_order_acquire);
        if (now - last_cmd > params_.cmd_timeout) {
          ELITE::vector6d_t hold{};
          for (int i = 0; i < 6; ++i) {
            hold[i] = joints[i];
          }
          robot_->writeservoj(hold, 0);
          last_cmd_time_.store(now, std::memory_order_release);
        }
      } else {
        // 手柄断连：立即锁定当前关节位置，不等待超时
        ELITE::vector6d_t hold{};
        for (int i = 0; i < 6; ++i) {
          hold[i] = joints[i];
        }
        robot_->writeservoj(hold, 0);
        last_cmd_time_.store(now, std::memory_order_release);
      }

      publishWorkspaceViz();
      rate.sleep();
    }
  }

  // 仿真模式：根据手柄目标位姿直接求 IK 并发布 /joint_states。
  void simLoop() {
    ros::Rate rate(std::max(1.0, params_.sim_rate));
    sensor_msgs::JointState js;
    js.name.assign(sim_joint_names_.begin(), sim_joint_names_.begin() + std::min<size_t>(6, sim_joint_names_.size()));
    js.position.resize(6);
    while (ros::ok() && !shutdown_) {
      robot_set::TCPState desired;
      ros::Time stamp;
      bool have_des = false;
      {
        std::lock_guard<std::mutex> lk(desired_mtx_);
        if (desired_valid_) {
          desired = mapped_desired_;
          stamp = t_desired_;
          have_des = true;
        }
      }
      if (!have_des || desired.position.size() < 6 ||
          (ros::Time::now() - stamp).toSec() > desired_timeout_sec_) {
        // 手柄断连或超时：发布上一帧关节角锁定位置，工作空间保持
        if (sim_have_solution_) {
          js.header.stamp = ros::Time::now();
          for (int i = 0; i < 6; ++i) {
            js.position[i] = sim_last_solution_[i];
          }
          joint_pub_.publish(js);
        }
        rate.sleep();
        continue;
      }

      Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
      target.translation() = Eigen::Vector3d(desired.position[0], desired.position[1], desired.position[2]);
      target.linear() = rotvecToRot(Eigen::Vector3d(desired.position[3], desired.position[4], desired.position[5]));

      std::vector<double> seed(6);
      if (sim_have_solution_ && sim_last_solution_.size() >= 6) {
        seed = sim_last_solution_;
      } else {
        for (int i = 0; i < 6; ++i) {
          seed[i] = sim_seed_[i];
        }
      }

      std::vector<double> sol;
      const bool ok = pose_ik_.solve(target, seed, sol, params_.sim_ik_timeout);
      if (!ok || sol.size() < 6) {
        ROS_WARN_THROTTLE(1.0, "robot_main(sim): IK failed");
        rate.sleep();
        continue;
      }

      if (sim_have_solution_) {
        for (size_t i = 0; i < 6 && i < sol.size() && i < sim_last_solution_.size(); ++i) {
          sol[i] = unwrapToClosest(sol[i], sim_last_solution_[i]);
        }
      }
      sim_last_solution_ = sol;
      sim_have_solution_ = true;

      js.header.stamp = ros::Time::now();
      for (int i = 0; i < 6; ++i) {
        js.position[i] = sol[i];
      }
      joint_pub_.publish(js);

      publishWorkspaceViz();
      rate.sleep();
    }
  }

  void publishWorkspaceViz() {
    if (!params_.workspace_viz_enable || !workspace_marker_pub_) {
      return;
    }
    const double now = ros::Time::now().toSec();
    if (now - last_workspace_pub_time_ < (1.0 / std::max(1.0, params_.workspace_viz_rate))) {
      return;
    }
    last_workspace_pub_time_ = now;

    Eigen::Vector3d drift_center;
    Eigen::Vector3d drift_size;
    Eigen::Vector3d work_center;
    Eigen::Vector3d work_size;
    pose_mapper_.getWorkspaceBoxes(drift_center, drift_size, work_center, work_size);

    visualization_msgs::Marker drift_box;
    drift_box.header.stamp = ros::Time::now();
    drift_box.header.frame_id = params_.workspace_viz_frame;
    drift_box.ns = "workspace_window";
    drift_box.id = 0;
    drift_box.type = visualization_msgs::Marker::CUBE;
    drift_box.action = visualization_msgs::Marker::ADD;
    drift_box.pose.position.x = drift_center.x();
    drift_box.pose.position.y = drift_center.y();
    drift_box.pose.position.z = drift_center.z();
    drift_box.pose.orientation.w = 1.0;
    drift_box.scale.x = drift_size.x();
    drift_box.scale.y = drift_size.y();
    drift_box.scale.z = drift_size.z();
    drift_box.color.r = 0.1f;
    drift_box.color.g = 0.6f;
    drift_box.color.b = 0.9f;
    drift_box.color.a = 0.15f;
    drift_box.lifetime = ros::Duration(0.0);
    workspace_marker_pub_.publish(drift_box);

    visualization_msgs::Marker work_box = drift_box;
    work_box.id = 1;
    work_box.pose.position.x = work_center.x();
    work_box.pose.position.y = work_center.y();
    work_box.pose.position.z = work_center.z();
    work_box.scale.x = work_size.x();
    work_box.scale.y = work_size.y();
    work_box.scale.z = work_size.z();
    work_box.color.a = 0.3f;
    workspace_marker_pub_.publish(work_box);
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
  // 若 drift_force_enable，叠加"漂移区弹簧力"：机械臂 TCP 超出工作空间 box 时，
  // 各超出轴独立施加指向 box 中心的弹簧力（box 各轴 work_pos/work_neg 独立）。
  void publishForce() {
    ros::Rate rate(params_.pub_hz);
    std_msgs::Float64MultiArray msg;
    msg.data.resize(6);
    while (ros::ok() && !shutdown_) {
      // 先在 state_mtx_ 下拷贝 force / tcp 缓存，避免与 sdkLoop 竞争
      double force_copy[6];
      double tcp_pos[3];
      {
        std::lock_guard<std::mutex> lk(state_mtx_);
        for (int i = 0; i < 6; ++i) {
          force_copy[i] = force_cache_[i];
        }
        for (int i = 0; i < 3; ++i) {
          tcp_pos[i] = tcp_cache_[i];
        }
      }
      for (int i = 0; i < 6; ++i) {
        msg.data[i] = force_copy[i];
      }

      // 漂移区弹簧力叠加（仅平移力分量 Fx/Fy/Fz）
      // 工作空间是矩形 box（各轴 work_pos/work_neg 独立），用 getWorkspaceBoxes
      // 逐轴算超出量；任一轴超出即在该轴方向施加指向 box 中心的弹簧力。
      if (params_.drift_force_enable) {
        Eigen::Vector3d drift_center, drift_size, work_center, work_size;
        pose_mapper_.getWorkspaceBoxes(drift_center, drift_size, work_center, work_size);
        const Eigen::Vector3d work_half = 0.5 * work_size;
        const Eigen::Vector3d r_pos(tcp_pos[0], tcp_pos[1], tcp_pos[2]);
        const Eigen::Vector3d delta = r_pos - work_center;
        // 各轴 |delta| - half：超出量；负值表示在 box 内，置 0
        const Eigen::Vector3d over = (delta.cwiseAbs() - work_half).cwiseMax(0.0);
        if (over.maxCoeff() > 0.0) {
          // 各轴独立：力方向 = -sign(delta)，大小 = k * over
          Eigen::Vector3d fvec;
          for (int i = 0; i < 3; ++i) {
            if (over[i] > 0.0) {
              fvec[i] = -params_.drift_force_k * over[i]
                        * (delta[i] >= 0.0 ? 1.0 : -1.0);
            } else {
              fvec[i] = 0.0;
            }
          }
          // 限幅：总力向量模长不超过 max
          const double fmag = fvec.norm();
          if (fmag > params_.drift_force_max) {
            fvec *= (params_.drift_force_max / std::max(fmag, 1e-9));
          }
          std::cout << fvec[0] << "," << fvec[1] << "," << fvec[2] << std::endl;
          for (int i = 0; i < 3; ++i) {
            msg.data[i] += fvec[i];
          }
        }
      }
      // std::cout << msg.data[0] << "," << msg.data[1] << "," << msg.data[2] << std::endl;
      force_pub_.publish(msg);
      rate.sleep();
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  RuntimeParams params_;
  PoseMapper pose_mapper_;
  PoseErrorController controller_;
  MoveItPoseIk pose_ik_;
  std::unique_ptr<EliteCSRobotSDK> robot_;

  ros::Subscriber haptic_sub_;
  ros::Publisher joint_pub_;
  ros::Publisher tcp_pub_;
  ros::Publisher force_pub_;
  ros::Publisher workspace_marker_pub_;

  std::thread sdk_thread_;
  std::thread pub_joint_thread_;
  std::thread pub_tcp_thread_;
  std::thread pub_force_thread_;
  std::thread sim_thread_;
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

  std::atomic<double> last_cmd_time_{0.0};

  std::vector<std::string> sim_joint_names_;
  std::array<double, 6> sim_seed_{};
  std::vector<double> sim_last_solution_;
  bool sim_have_solution_{false};

  double last_workspace_pub_time_{0.0};

  // ---- 表面感知 ----
  SurfaceSampleProcessor surface_processor_;
  std::unique_ptr<actionlib::SimpleActionClient<realsense_set::ScanEnvironmentAction>> scan_client_;
  std::atomic<bool> scan_requested_{false};
  std::atomic<bool> scan_done_{false};
  enum class ScanState { NORMAL, LOCKING, SCANNING };
  ScanState scan_state_{ScanState::NORMAL};
  ELITE::vector6d_t lock_joints_{};
  ros::Time lock_start_time_;
  std::thread keyboard_thread_;

  // 表面安全限制缓存：记录最近一次成功的表面参考点和指向内部的法向量。
  // 当 actual_pos 超出采样点云覆盖范围、computeBlendedNormal 返回 false 时，
  // 仍可据此把机械臂限在安全边界上，而不是放任越界。
  Eigen::Vector3d safety_surface_pt_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d safety_normal_{Eigen::Vector3d::UnitZ()};  // 已翻转，指向工作空间内部
  bool safety_ref_valid_{false};
  ros::Time safety_ref_time_;  // 最近一次成功 in-region 查询的时间，用于出区域后的衰减放手
};

int main(int argc, char** argv) {
  // 程序入口：初始化 ROS 节点，启动异步 spinner，并运行主控制节点。
  ros::init(argc, argv, "robot_main_node");
  setlocale(LC_ALL,"");
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
