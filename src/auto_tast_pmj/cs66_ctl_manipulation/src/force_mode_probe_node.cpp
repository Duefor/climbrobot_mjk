#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include "cs66_robot_controller.h"
#include "cs66_follow_joint_traj_server.h"

namespace {

std::vector<double> GetDoubleVectorParam(
    const ros::NodeHandle& pnh,
    const std::string& name,
    const std::vector<double>& fallback) {
  std::vector<double> value;
  if (!pnh.getParam(name, value)) {
    return fallback;
  }
  if (value.size() != fallback.size()) {
    ROS_WARN("~%s size is %zu, expected %zu. Using fallback.",
             name.c_str(), value.size(), fallback.size());
    return fallback;
  }
  return value;
}

std::string MoveItCodeToString(const moveit::planning_interface::MoveItErrorCode& code) {
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
    default:
      return "MOVEIT_ERROR_" + std::to_string(code.val);
  }
}

ELITE::vector6d_t PoseToEliteAxisAngle(const geometry_msgs::Pose& pose) {
  ELITE::vector6d_t frame;
  frame[0] = pose.position.x;
  frame[1] = pose.position.y;
  frame[2] = pose.position.z;

  tf2::Quaternion q(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w);
  q.normalize();

  double w = std::max(-1.0, std::min(1.0, q.w()));
  const double angle = 2.0 * std::acos(w);
  const double s = std::sqrt(std::max(0.0, 1.0 - w * w));
  if (s < 1e-8) {
    frame[3] = 0.0;
    frame[4] = 0.0;
    frame[5] = 0.0;
  } else {
    frame[3] = (q.x() / s) * angle;
    frame[4] = (q.y() / s) * angle;
    frame[5] = (q.z() / s) * angle;
  }
  return frame;
}

class ForceModeGuard {
 public:
  explicit ForceModeGuard(const std::shared_ptr<CS66RobotController>& controller)
      : controller_(controller), active_(false) {}

  bool Start(const ELITE::vector6d_t& frame,
             const ELITE::vector6int32_t& selection,
             const ELITE::vector6d_t& wrench,
             const ELITE::vector6d_t& limits) {
    ros::Duration(0.5).sleep();
    active_ = controller_ && controller_->startForceMode(frame, selection, wrench, limits);
    ros::Duration(0.5).sleep();
    return active_;
  }

  bool Stop() {
    if (!active_) {
      return true;
    }
    active_ = false;
    return controller_->endForceMode();
  }

  ~ForceModeGuard() {
    if (active_) {
      controller_->endForceMode();
    }
  }

 private:
  std::shared_ptr<CS66RobotController> controller_;
  bool active_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "force_mode_probe_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  auto controller = std::make_shared<CS66RobotController>(nh, pnh);
  ros::Duration(1.0).sleep();

  CS66FollowTrajectoryServer follow_server(nh, controller);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("arm_group");
  move_group.setPoseReferenceFrame("base_link");
  move_group.setEndEffectorLink("ee_link");

  double planning_time = 5.0;
  double velocity_scale = 0.3;
  double acceleration_scale = 0.5;
  double cartesian_step = 0.005;
  double dx = 0.0;
  double dy = -0.3;
  double dz = 0.0;
  double force_z = 8.0;
  double force_limit_z = 0.03;
  double settle_before_execute = 1.5;
  double post_hold = 2.0;
  bool require_enter = true;

  pnh.param("planning_time", planning_time, planning_time);
  pnh.param("velocity_scale", velocity_scale, velocity_scale);
  pnh.param("acceleration_scale", acceleration_scale, acceleration_scale);
  pnh.param("cartesian_step", cartesian_step, cartesian_step);
  pnh.param("cartesian_dx", dx, dx);
  pnh.param("cartesian_dy", dy, dy);
  pnh.param("cartesian_dz", dz, dz);
  pnh.param("force_z", force_z, force_z);
  pnh.param("force_limit_z", force_limit_z, force_limit_z);
  pnh.param("settle_before_execute", settle_before_execute, settle_before_execute);
  pnh.param("post_hold", post_hold, post_hold);
  pnh.param("require_enter", require_enter, require_enter);

  const std::vector<double> default_start_joints = {0.7, -1.63, -2.09, -1.01, 1.57, -0.88};
  const std::vector<double> start_joints =
      GetDoubleVectorParam(pnh, "start_joints", default_start_joints);

  move_group.setPlanningTime(planning_time);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(velocity_scale);
  move_group.setMaxAccelerationScalingFactor(acceleration_scale);

  ROS_INFO("Force probe params: force_z=%.3f force_limit_z=%.3f line_delta=[%.3f %.3f %.3f]",
           force_z, force_limit_z, dx, dy, dz);

  if (require_enter) {
    std::cout << "Press Enter to move to the force probe start joints..." << std::endl;
    std::cin.get();
  }

  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget(start_joints);
  const auto start_result = move_group.move();
  ROS_INFO("Move to start result=%s (%d)",
           MoveItCodeToString(start_result).c_str(), start_result.val);
  if (start_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Failed to move to force probe start joints.");
    return 1;
  }

  ros::Duration(0.5).sleep();
  geometry_msgs::Pose start_pose = move_group.getCurrentPose("ee_link").pose;
  geometry_msgs::Pose end_pose = start_pose;
  end_pose.position.x += dx;
  end_pose.position.y += dy;
  end_pose.position.z += dz;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(end_pose);

  move_group.setStartStateToCurrentState();
  moveit_msgs::RobotTrajectory trajectory;
  const double fraction = move_group.computeCartesianPath(waypoints, cartesian_step, trajectory);
  ROS_INFO("Cartesian line computed: fraction=%.6f points=%zu",
           fraction, trajectory.joint_trajectory.points.size());
  if (trajectory.joint_trajectory.points.empty()) {
    ROS_ERROR("Cartesian line trajectory is empty.");
    return 1;
  }
  if (fraction < 0.99) {
    ROS_WARN("Cartesian line is incomplete: %.2f%%", fraction * 100.0);
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;

  if (require_enter) {
    std::cout << "Press Enter to start force mode and execute the straight line..." << std::endl;
    std::cin.get();
  }

  auto tcp_pose_msg = ros::topic::waitForMessage<geometry_msgs::Pose>(
      "/tcp_pose", nh, ros::Duration(1.0));
  if (!tcp_pose_msg) {
    ROS_ERROR("Failed to read /tcp_pose for force mode reference frame.");
    return 1;
  }

  const ELITE::vector6d_t force_ref_frame = PoseToEliteAxisAngle(*tcp_pose_msg);
  const ELITE::vector6int32_t force_selection_vector = {0, 0, 1, 0, 0, 0};
  const ELITE::vector6d_t force_wrench = {0.0, 0.0, force_z, 0.0, 0.0, 0.0};
  const ELITE::vector6d_t force_limits = {0.0, 0.0, force_limit_z, 0.0, 0.0, 0.0};

  ROS_INFO("Starting force mode: ref=[%.4f %.4f %.4f %.4f %.4f %.4f] wrench_z=%.3f limit_z=%.3f",
           force_ref_frame[0], force_ref_frame[1], force_ref_frame[2],
           force_ref_frame[3], force_ref_frame[4], force_ref_frame[5],
           force_z, force_limit_z);

  ForceModeGuard force_guard(controller);
  if (!force_guard.Start(force_ref_frame, force_selection_vector, force_wrench, force_limits)) {
    ROS_ERROR("Failed to start force mode.");
    return 1;
  }

  ros::Duration(settle_before_execute).sleep();

  const geometry_msgs::Pose pose_before = move_group.getCurrentPose("ee_link").pose;
  const auto exec_result = move_group.execute(plan);
  const geometry_msgs::Pose pose_after = move_group.getCurrentPose("ee_link").pose;
  ROS_INFO("Straight line execute result=%s (%d)",
           MoveItCodeToString(exec_result).c_str(), exec_result.val);
  ROS_INFO("TCP delta while force mode active: dx=%.5f dy=%.5f dz=%.5f",
           pose_after.position.x - pose_before.position.x,
           pose_after.position.y - pose_before.position.y,
           pose_after.position.z - pose_before.position.z);

  ros::Duration(post_hold).sleep();

  if (!force_guard.Stop()) {
    ROS_ERROR("Failed to stop force mode.");
    return 1;
  }

  ROS_INFO("Force mode probe finished.");
  return exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS ? 0 : 1;
}
