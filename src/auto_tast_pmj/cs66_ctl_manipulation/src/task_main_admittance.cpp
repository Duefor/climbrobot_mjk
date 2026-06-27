#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "cs66_robot_controller.h"
#include "cs66_follow_joint_traj_server.h"

#include "mv3d_rgbd_ros/DetectSteelStamp.h"

namespace {

constexpr double kCartesianStepMeters = 0.005;
const char* kFirstApproachPlannerId = "RRTstar";

struct AdmittanceParams {
  // 期望的 TCP-Z 力传感器读数，单位 N。
  // 当前末端朝下且力传感器坐标系与 TCP 坐标系重合，期望 8N 接触力时读数应稳定在 -8N。
  double target_force_z = -8.0;

  // 力误差方向系数。默认 measured - target 为正时，沿 TCP 自身 +Z 方向补偿。
  // 如果实机发现补偿方向反了，只需要把该参数设为 -1。
  double force_sign = 1.0;

  // 导纳模型虚拟质量。越大，Z 补偿响应越慢、越稳。
  double mass = 1.0;

  // 导纳模型虚拟阻尼。越大，Z 方向速度越容易被抑制，振荡越少。
  double damping = 30.0;

  // 导纳模型虚拟刚度。越大，Z 补偿位移越倾向回到 0，长期偏移越小。
  double stiffness = 80.0;

  // 力传感器低通滤波系数，范围 [0, 1]。越大越相信最新力值，越小越平滑。
  double force_filter_alpha = 0.15;

  // 手动导纳控制周期，单位 s。第一版默认 10ms，优先稳定。
  double control_period = 0.01;

  // 名义 TCP 轨迹推进速度，单位 m/s。
  double nominal_tcp_speed = 0.02;

  // TCP 局部 Z 方向最大补偿位移，单位 m。
  double max_z_offset = 0.02;

  // TCP 局部 Z 方向最大补偿速度，单位 m/s。
  double max_z_velocity = 0.01;

  // 单个控制周期内允许的最大关节角变化，单位 rad，防止 IK 跳解。
  double max_joint_step = 0.03;

  // 力安全阈值，单位 N。滤波后 TCP-Z 力绝对值超过该值则立即停止。
  double force_abort_threshold = 50.0;

  // /tcp_pose、/tcp_force、/joint_states 的最大允许数据超时，单位 s。
  double state_timeout = 0.2;

  // 连续 IK 失败允许次数，超过后停止执行。
  int max_consecutive_ik_failures = 3;
};

struct RealtimeSnapshot {
  geometry_msgs::Pose tcp_pose;
  std::array<double, 6> tcp_force{};
  std::vector<double> joints;
};

double Clamp(double value, double lo, double hi) {
  return std::max(lo, std::min(hi, value));
}

double PoseDistance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b) {
  const double dx = b.position.x - a.position.x;
  const double dy = b.position.y - a.position.y;
  const double dz = b.position.z - a.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

tf2::Quaternion PoseQuaternion(const geometry_msgs::Pose& pose) {
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  q.normalize();
  return q;
}

geometry_msgs::Pose InterpolatePose(
    const geometry_msgs::Pose& a,
    const geometry_msgs::Pose& b,
    double t) {
  t = Clamp(t, 0.0, 1.0);

  geometry_msgs::Pose out;
  out.position.x = a.position.x + (b.position.x - a.position.x) * t;
  out.position.y = a.position.y + (b.position.y - a.position.y) * t;
  out.position.z = a.position.z + (b.position.z - a.position.z) * t;

  tf2::Quaternion qa = PoseQuaternion(a);
  tf2::Quaternion qb = PoseQuaternion(b);
  tf2::Quaternion q = qa.slerp(qb, t);
  q.normalize();
  out.orientation = tf2::toMsg(q);
  return out;
}

geometry_msgs::Pose PoseAtDistance(
    const std::vector<geometry_msgs::Pose>& poses,
    const std::vector<double>& cumulative_lengths,
    double distance) {
  if (poses.empty()) {
    return geometry_msgs::Pose();
  }
  if (poses.size() == 1 || distance <= 0.0) {
    return poses.front();
  }
  if (distance >= cumulative_lengths.back()) {
    return poses.back();
  }

  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    const double seg_start = cumulative_lengths[i];
    const double seg_end = cumulative_lengths[i + 1];
    if (distance <= seg_end) {
      const double seg_len = seg_end - seg_start;
      const double t = seg_len > 1e-9 ? (distance - seg_start) / seg_len : 0.0;
      return InterpolatePose(poses[i], poses[i + 1], t);
    }
  }

  return poses.back();
}

std::vector<double> BuildCumulativeLengths(const std::vector<geometry_msgs::Pose>& poses) {
  std::vector<double> cumulative;
  cumulative.reserve(poses.size());
  cumulative.push_back(0.0);
  for (size_t i = 1; i < poses.size(); ++i) {
    cumulative.push_back(cumulative.back() + PoseDistance(poses[i - 1], poses[i]));
  }
  return cumulative;
}

ELITE::vector6d_t ToEliteJoints(const std::vector<double>& joints) {
  ELITE::vector6d_t out = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  for (size_t i = 0; i < 6 && i < joints.size(); ++i) {
    out[i] = joints[i];
  }
  return out;
}

double MaxAbsJointDelta(const std::vector<double>& a, const std::vector<double>& b) {
  const size_t n = std::min(a.size(), b.size());
  double max_delta = 0.0;
  for (size_t i = 0; i < n; ++i) {
    max_delta = std::max(max_delta, std::abs(a[i] - b[i]));
  }
  return max_delta;
}

std::string MoveItErrorCodeToString(const moveit::planning_interface::MoveItErrorCode& code) {
  switch (code.val) {
    case moveit_msgs::MoveItErrorCodes::SUCCESS:
      return "SUCCESS";
    case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
      return "PLANNING_FAILED";
    case moveit_msgs::MoveItErrorCodes::CONTROL_FAILED:
      return "CONTROL_FAILED";
    case moveit_msgs::MoveItErrorCodes::TIMED_OUT:
      return "TIMED_OUT";
    case moveit_msgs::MoveItErrorCodes::PREEMPTED:
      return "PREEMPTED";
    case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
      return "NO_IK_SOLUTION";
    default:
      return "MOVEIT_ERROR_" + std::to_string(code.val);
  }
}

void LoadAdmittanceParams(const ros::NodeHandle& pnh, AdmittanceParams& params) {
  pnh.param("target_force_z", params.target_force_z, params.target_force_z);
  pnh.param("force_sign", params.force_sign, params.force_sign);
  pnh.param("admittance_mass", params.mass, params.mass);
  pnh.param("admittance_damping", params.damping, params.damping);
  pnh.param("admittance_stiffness", params.stiffness, params.stiffness);
  pnh.param("force_filter_alpha", params.force_filter_alpha, params.force_filter_alpha);
  pnh.param("control_period", params.control_period, params.control_period);
  pnh.param("nominal_tcp_speed", params.nominal_tcp_speed, params.nominal_tcp_speed);
  pnh.param("max_z_offset", params.max_z_offset, params.max_z_offset);
  pnh.param("max_z_velocity", params.max_z_velocity, params.max_z_velocity);
  pnh.param("max_joint_step", params.max_joint_step, params.max_joint_step);
  pnh.param("force_abort_threshold", params.force_abort_threshold, params.force_abort_threshold);
  pnh.param("state_timeout", params.state_timeout, params.state_timeout);
  pnh.param("max_consecutive_ik_failures",
            params.max_consecutive_ik_failures,
            params.max_consecutive_ik_failures);

  params.force_filter_alpha = Clamp(params.force_filter_alpha, 0.0, 1.0);
  params.control_period = std::max(0.002, params.control_period);
  params.nominal_tcp_speed = std::max(1e-4, params.nominal_tcp_speed);
  params.mass = std::max(1e-6, params.mass);
}

class RobotRealtimeStateCache {
 public:
  RobotRealtimeStateCache(ros::NodeHandle& nh,
                          const std::string& tcp_pose_topic,
                          const std::string& tcp_force_topic,
                          const std::string& joint_state_topic) {
    tcp_pose_sub_ = nh.subscribe(tcp_pose_topic, 10, &RobotRealtimeStateCache::TcpPoseCb, this);
    tcp_force_sub_ = nh.subscribe(tcp_force_topic, 10, &RobotRealtimeStateCache::TcpForceCb, this);
    joint_state_sub_ = nh.subscribe(joint_state_topic, 10, &RobotRealtimeStateCache::JointStateCb, this);
  }

  bool GetSnapshot(const std::vector<std::string>& joint_names,
                   double timeout,
                   RealtimeSnapshot& snapshot,
                   std::string& reason) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::Time now = ros::Time::now();

    if (!has_tcp_pose_) {
      reason = "no tcp pose";
      return false;
    }
    if (!has_tcp_force_) {
      reason = "no tcp force";
      return false;
    }
    if (!has_joint_state_) {
      reason = "no joint state";
      return false;
    }
    if ((now - tcp_pose_stamp_).toSec() > timeout) {
      reason = "tcp pose timeout";
      return false;
    }
    if ((now - tcp_force_stamp_).toSec() > timeout) {
      reason = "tcp force timeout";
      return false;
    }
    if ((now - joint_state_stamp_).toSec() > timeout) {
      reason = "joint state timeout";
      return false;
    }

    snapshot.tcp_pose = tcp_pose_;
    snapshot.tcp_force = tcp_force_;
    snapshot.joints.assign(joint_names.size(), 0.0);

    for (size_t i = 0; i < joint_names.size(); ++i) {
      const auto it = std::find(joint_state_.name.begin(), joint_state_.name.end(), joint_names[i]);
      if (it == joint_state_.name.end()) {
        reason = "joint state missing " + joint_names[i];
        return false;
      }
      const size_t idx = static_cast<size_t>(std::distance(joint_state_.name.begin(), it));
      if (idx >= joint_state_.position.size()) {
        reason = "joint state position missing " + joint_names[i];
        return false;
      }
      snapshot.joints[i] = joint_state_.position[idx];
    }

    return true;
  }

 private:
  void TcpPoseCb(const geometry_msgs::Pose::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    tcp_pose_ = *msg;
    tcp_pose_stamp_ = ros::Time::now();
    has_tcp_pose_ = true;
  }

  void TcpForceCb(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 6) {
      ROS_WARN_THROTTLE(1.0, "tcp force message has fewer than 6 values");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < 6; ++i) {
      tcp_force_[i] = msg->data[i];
    }
    tcp_force_stamp_ = ros::Time::now();
    has_tcp_force_ = true;
  }

  void JointStateCb(const sensor_msgs::JointState::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    joint_state_ = *msg;
    joint_state_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_joint_state_ = true;
  }

  ros::Subscriber tcp_pose_sub_;
  ros::Subscriber tcp_force_sub_;
  ros::Subscriber joint_state_sub_;

  mutable std::mutex mutex_;
  geometry_msgs::Pose tcp_pose_;
  std::array<double, 6> tcp_force_{};
  sensor_msgs::JointState joint_state_;
  ros::Time tcp_pose_stamp_;
  ros::Time tcp_force_stamp_;
  ros::Time joint_state_stamp_;
  bool has_tcp_pose_ = false;
  bool has_tcp_force_ = false;
  bool has_joint_state_ = false;
};

class JointServoAdmittanceExecutor {
 public:
  JointServoAdmittanceExecutor(
      const std::shared_ptr<CS66RobotController>& controller,
      moveit::planning_interface::MoveGroupInterface& move_group,
      RobotRealtimeStateCache& state_cache,
      const AdmittanceParams& params)
      : controller_(controller),
        move_group_(move_group),
        state_cache_(state_cache),
        params_(params) {}

  bool Execute(const std::vector<geometry_msgs::Pose>& nominal_poses) {
    if (!controller_) {
      ROS_ERROR("Admittance executor has no controller");
      return false;
    }
    if (nominal_poses.size() < 2) {
      ROS_ERROR("Admittance executor needs at least two nominal poses");
      return false;
    }

    moveit::core::RobotStatePtr current_state = move_group_.getCurrentState();
    if (!current_state) {
      ROS_ERROR("Failed to get current MoveIt state");
      return false;
    }
    const moveit::core::JointModelGroup* joint_model_group =
        current_state->getJointModelGroup(move_group_.getName());
    if (!joint_model_group) {
      ROS_ERROR("Failed to get joint model group: %s", move_group_.getName().c_str());
      return false;
    }
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    RealtimeSnapshot snapshot;
    std::string reason;
    if (!state_cache_.GetSnapshot(joint_names, params_.state_timeout, snapshot, reason)) {
      ROS_ERROR("Realtime state is not ready: %s", reason.c_str());
      return false;
    }

    std::vector<double> seed_joints = snapshot.joints;
    std::vector<double> last_cmd_joints = snapshot.joints;
    double filtered_force_z = snapshot.tcp_force[2];
    double z_offset = 0.0;
    double z_dot = 0.0;
    int consecutive_ik_failures = 0;

    const std::vector<double> cumulative_lengths = BuildCumulativeLengths(nominal_poses);
    const double total_length = cumulative_lengths.back();
    if (total_length < 1e-6) {
      ROS_ERROR("Nominal admittance path length is too small");
      return false;
    }
    const double total_duration = total_length / params_.nominal_tcp_speed;

    ROS_INFO("Manual admittance execution starts: length=%.4f duration=%.3f target_fz=%.3f",
             total_length, total_duration, params_.target_force_z);

    ros::Rate rate(1.0 / params_.control_period);
    const ros::Time start_time = ros::Time::now();
    size_t cycle = 0;

    while (ros::ok()) {
      const double elapsed = (ros::Time::now() - start_time).toSec();
      const double path_distance = std::min(total_length, elapsed * params_.nominal_tcp_speed);
      const geometry_msgs::Pose nominal_pose =
          PoseAtDistance(nominal_poses, cumulative_lengths, path_distance);

      if (!state_cache_.GetSnapshot(joint_names, params_.state_timeout, snapshot, reason)) {
        ROS_ERROR("Realtime state lost during admittance execution: %s", reason.c_str());
        controller_->WriteIdleOnce();
        return false;
      }

      filtered_force_z =
          params_.force_filter_alpha * snapshot.tcp_force[2] +
          (1.0 - params_.force_filter_alpha) * filtered_force_z;

      if (std::abs(filtered_force_z) > params_.force_abort_threshold) {
        ROS_ERROR("Force abort: filtered_fz=%.3f threshold=%.3f",
                  filtered_force_z, params_.force_abort_threshold);
        controller_->WriteIdleOnce();
        return false;
      }

      // 符号约定：
      // 末端朝下时，期望 8N 接触力对应传感器 TCP-Z 读数约 -8N。
      // 若当前 filtered_force_z=0N，说明还没有达到接触力：
      //   force_error = 0 - (-8) = +8
      // 于是 z_offset 沿当前名义 TCP 的局部 +Z 方向增大，继续向工件补偿。
      const double force_error =
          params_.force_sign * (filtered_force_z - params_.target_force_z);
      const double z_ddot =
          (force_error - params_.damping * z_dot - params_.stiffness * z_offset) /
          params_.mass;

      z_dot = Clamp(z_dot + z_ddot * params_.control_period,
                    -params_.max_z_velocity,
                    params_.max_z_velocity);
      z_offset = Clamp(z_offset + z_dot * params_.control_period,
                       -params_.max_z_offset,
                       params_.max_z_offset);

      tf2::Quaternion q = PoseQuaternion(nominal_pose);
      const tf2::Vector3 local_z_axis = tf2::quatRotate(q, tf2::Vector3(0.0, 0.0, 1.0));

      geometry_msgs::Pose cmd_pose = nominal_pose;
      cmd_pose.position.x += z_offset * local_z_axis.x();
      cmd_pose.position.y += z_offset * local_z_axis.y();
      cmd_pose.position.z += z_offset * local_z_axis.z();

      moveit::core::RobotState ik_state(*current_state);
      ik_state.setJointGroupPositions(joint_model_group, seed_joints);
      ik_state.update();

      const bool found_ik = ik_state.setFromIK(
          joint_model_group, cmd_pose, move_group_.getEndEffectorLink(), 0.005);
      if (!found_ik) {
        ++consecutive_ik_failures;
        ROS_WARN("Admittance IK failed (%d/%d)",
                 consecutive_ik_failures, params_.max_consecutive_ik_failures);
        if (consecutive_ik_failures >= params_.max_consecutive_ik_failures) {
          controller_->WriteIdleOnce();
          return false;
        }
        rate.sleep();
        continue;
      }
      consecutive_ik_failures = 0;

      std::vector<double> q_cmd;
      ik_state.copyJointGroupPositions(joint_model_group, q_cmd);
      const double max_joint_delta = MaxAbsJointDelta(last_cmd_joints, q_cmd);
      if (max_joint_delta > params_.max_joint_step) {
        ROS_ERROR("Joint step abort: max_delta=%.5f threshold=%.5f",
                  max_joint_delta, params_.max_joint_step);
        controller_->WriteIdleOnce();
        return false;
      }

      if (!controller_->WriteServojJointOnce(ToEliteJoints(q_cmd), 100)) {
        controller_->WriteIdleOnce();
        return false;
      }

      seed_joints = q_cmd;
      last_cmd_joints = q_cmd;

      if (cycle % 25 == 0) {
        ROS_INFO("admittance t=%.2f/%.2f fz=%.3f filtered=%.3f z_offset=%.5f z_dot=%.5f joint_step=%.5f",
                 elapsed,
                 total_duration,
                 snapshot.tcp_force[2],
                 filtered_force_z,
                 z_offset,
                 z_dot,
                 max_joint_delta);
      }
      ++cycle;

      if (path_distance >= total_length) {
        break;
      }

      rate.sleep();
    }

    controller_->WriteIdleOnce();
    ROS_INFO("Manual admittance execution finished.");
    return ros::ok();
  }

 private:
  std::shared_ptr<CS66RobotController> controller_;
  moveit::planning_interface::MoveGroupInterface& move_group_;
  RobotRealtimeStateCache& state_cache_;
  AdmittanceParams params_;
};

void PrintTargetPoses(const std::vector<geometry_msgs::Pose>& target_poses) {
  ROS_INFO("=== Debugging Target Poses ===");
  for (size_t i = 0; i < target_poses.size(); ++i) {
    const auto& pose = target_poses[i];
    ROS_INFO("Pose[%lu] Position: [x: %.4f, y: %.4f, z: %.4f]",
             i, pose.position.x, pose.position.y, pose.position.z);
    ROS_INFO("Pose[%lu] Quat: [x: %.4f, y: %.4f, z: %.4f, w: %.4f]",
             i, pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w);
  }
}

bool MoveToPoseWithOptimizingPlanner(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::Pose& target_pose) {
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      current_state->getJointModelGroup(move_group.getName());

  bool found_ik = current_state->setFromIK(joint_model_group, target_pose, 0.1);
  if (!found_ik) {
    ROS_ERROR("IK Solver failed to find a valid joint solution for the first approach point.");
    return false;
  }

  std::vector<double> target_joint_values;
  current_state->copyJointGroupPositions(joint_model_group, target_joint_values);

  move_group.setStartStateToCurrentState();
  move_group.setPlannerId(kFirstApproachPlannerId);
  move_group.setJointValueTarget(target_joint_values);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto plan_result = move_group.plan(plan);
  if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Failed to plan motion to the first approach point with planner %s.",
              kFirstApproachPlannerId);
    move_group.clearPoseTargets();
    return false;
  }

  const auto exec_result = move_group.execute(plan);
  if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Failed to execute motion to the first approach point.");
    move_group.clearPoseTargets();
    return false;
  }

  move_group.clearPoseTargets();
  return true;
}

std::vector<geometry_msgs::Pose> BuildTrackingWaypoints(
    const std::vector<geometry_msgs::Pose>& target_poses) {
  std::vector<geometry_msgs::Pose> waypoints;
  if (target_poses.empty()) {
    return waypoints;
  }
  waypoints.reserve(target_poses.size() > 1 ? target_poses.size() - 1 : 1);
  for (size_t i = 1; i < target_poses.size(); ++i) {
    waypoints.push_back(target_poses[i]);
  }
  if (waypoints.empty()) {
    waypoints.push_back(target_poses.front());
  }
  return waypoints;
}

void PrintTrajectoryDebug(
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::RobotTrajectory& trajectory) {
  const auto& points = trajectory.joint_trajectory.points;
  ROS_INFO("[Cartesian debug] group=%s points=%zu",
           move_group.getName().c_str(), points.size());
  if (points.empty()) {
    ROS_ERROR("[Cartesian debug] trajectory points are empty.");
    return;
  }
  ROS_INFO("[Cartesian debug] duration=%.6f",
           points.back().time_from_start.toSec() - points.front().time_from_start.toSec());
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "steel_task_admittance_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  auto controller = std::make_shared<CS66RobotController>(nh, pnh);
  ros::Duration(1.0).sleep();

  CS66FollowTrajectoryServer moveit_server(nh, controller);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("arm_group");
  move_group.setPoseReferenceFrame("base_link");
  move_group.setEndEffectorLink("ee_link");
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.1);

  std::string tcp_pose_topic = "/tcp_pose";
  std::string tcp_force_topic = "/tcp_force";
  std::string joint_state_topic = "/joint_states";
  pnh.param("tcp_pose_topic", tcp_pose_topic, tcp_pose_topic);
  pnh.param("tcp_force_topic", tcp_force_topic, tcp_force_topic);
  pnh.param("joint_state_topic", joint_state_topic, joint_state_topic);

  AdmittanceParams admittance_params;
  LoadAdmittanceParams(pnh, admittance_params);

  RobotRealtimeStateCache state_cache(nh, tcp_pose_topic, tcp_force_topic, joint_state_topic);

  ROS_INFO("Admittance params: target_fz=%.3f sign=%.1f M=%.3f B=%.3f K=%.3f dt=%.4f speed=%.4f",
           admittance_params.target_force_z,
           admittance_params.force_sign,
           admittance_params.mass,
           admittance_params.damping,
           admittance_params.stiffness,
           admittance_params.control_period,
           admittance_params.nominal_tcp_speed);

  const std::vector<double> observation_joints = {0.03, -0.96, -1.95, -1.81, 1.57, -1.55};
  const std::vector<double> shouna_joints = {0.04, -0.11, -2.7, -1.24, 1.45, -0.37};

  ROS_INFO("Step 1: Go observation pose");
  std::cout << "Press Enter to move to observation pose..." << std::endl;
  std::cin.get();
  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget(observation_joints);
  auto move_result = move_group.move();
  if (move_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Failed to move to observation pose: %s",
              MoveItErrorCodeToString(move_result).c_str());
    return 1;
  }

  std::cout << "Press Enter to start OCR detection..." << std::endl;
  std::cin.get();

  ROS_INFO("Step 2: OCR Detection");
  ros::ServiceClient ocr_client =
      nh.serviceClient<mv3d_rgbd_ros::DetectSteelStamp>("get_steel_stamp_location");
  mv3d_rgbd_ros::DetectSteelStamp srv;
  if (!ocr_client.call(srv)) {
    ROS_ERROR("Failed to call OCR service");
    return 1;
  }
  if (!srv.response.success) {
    ROS_ERROR("OCR service returned failure: %s", srv.response.message.c_str());
    return 1;
  }

  const std::vector<geometry_msgs::Pose> target_poses = srv.response.poses;
  if (target_poses.size() < 2) {
    ROS_ERROR("OCR service returned too few target poses: %lu", target_poses.size());
    return 1;
  }
  ROS_INFO("Received %lu poses from OCR node.", target_poses.size());
  PrintTargetPoses(target_poses);

  ROS_INFO("Step 3: Move to the first pose returned by OCR trajectory");
  std::cout << "Press Enter to move to the first target pose..." << std::endl;
  std::cin.get();
  if (!MoveToPoseWithOptimizingPlanner(move_group, target_poses.front())) {
    return 1;
  }

  ros::Duration(0.5).sleep();

  ROS_INFO("Step 4: Build nominal cartesian trajectory");
  std::vector<geometry_msgs::Pose> waypoints = BuildTrackingWaypoints(target_poses);
  move_group.setStartStateToCurrentState();
  moveit_msgs::RobotTrajectory trajectory;
  const double fraction = move_group.computeCartesianPath(
      waypoints, kCartesianStepMeters, trajectory);
  ROS_INFO("[Cartesian debug] computeCartesianPath fraction=%.6f", fraction);
  PrintTrajectoryDebug(move_group, trajectory);
  if (fraction < 0.9) {
    ROS_WARN("Cartesian path incomplete (%.2f%%).", fraction * 100.0);
  }
  if (trajectory.joint_trajectory.points.empty()) {
    ROS_ERROR("Refusing to run admittance control with empty nominal trajectory.");
    return 1;
  }

  std::cout << "Press Enter to start manual joint-servo admittance execution..." << std::endl;
  std::cin.get();

  JointServoAdmittanceExecutor executor(controller, move_group, state_cache, admittance_params);
  if (!executor.Execute(target_poses)) {
    ROS_ERROR("Manual admittance execution failed.");
    return 1;
  }

  ros::Duration(2.0).sleep();

  ROS_INFO("Step 5: Return to safe stow pose");
  std::cout << "Press Enter to move back to safe pose..." << std::endl;
  std::cin.get();
  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget(shouna_joints);
  move_result = move_group.move();
  if (move_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Failed to move to safe pose: %s",
              MoveItErrorCodeToString(move_result).c_str());
    return 1;
  }

  ROS_INFO("task_main_admittance finished.");
  return 0;
}
