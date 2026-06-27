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

  AdmittanceForceControlParams admittance_params;
  pnh.param("target_force_z", admittance_params.target_force_z, admittance_params.target_force_z);
  pnh.param("force_sign", admittance_params.force_sign, admittance_params.force_sign);
  pnh.param("admittance_mass", admittance_params.admittance_mass, admittance_params.admittance_mass);
  pnh.param("admittance_damping", admittance_params.admittance_damping, admittance_params.admittance_damping);
  pnh.param("admittance_stiffness", admittance_params.admittance_stiffness, admittance_params.admittance_stiffness);
  pnh.param("force_filter_alpha", admittance_params.force_filter_alpha, admittance_params.force_filter_alpha);
  pnh.param("control_period", admittance_params.control_period, admittance_params.control_period);
  pnh.param("nominal_tcp_speed", admittance_params.nominal_tcp_speed, admittance_params.nominal_tcp_speed);
  pnh.param("max_z_offset", admittance_params.max_z_offset, admittance_params.max_z_offset);
  pnh.param("max_z_velocity", admittance_params.max_z_velocity, admittance_params.max_z_velocity);
  pnh.param("max_joint_step", admittance_params.max_joint_step, admittance_params.max_joint_step);
  pnh.param("force_abort_threshold", admittance_params.force_abort_threshold, admittance_params.force_abort_threshold);
  pnh.param("state_timeout", admittance_params.state_timeout, admittance_params.state_timeout);
  pnh.param("max_consecutive_ik_failures",
            admittance_params.max_consecutive_ik_failures,
            admittance_params.max_consecutive_ik_failures);

  ROS_INFO("Admittance params: target_fz=%.3f sign=%.1f M=%.3f B=%.3f K=%.3f dt=%.4f speed=%.4f",
           admittance_params.target_force_z,
           admittance_params.force_sign,
           admittance_params.admittance_mass,
           admittance_params.admittance_damping,
           admittance_params.admittance_stiffness,
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

  if (!controller->RunJointServoAdmittanceTrajectory(target_poses, move_group, admittance_params)) {
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
