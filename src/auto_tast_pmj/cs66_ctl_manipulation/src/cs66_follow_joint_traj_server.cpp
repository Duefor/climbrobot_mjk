#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>
#include <Elite/DataType.hpp>
#include "cs66_robot_controller.h"
#include "cs66_follow_joint_traj_server.h"

using FollowAction = control_msgs::FollowJointTrajectoryAction;
using FollowGoalConstPtr = control_msgs::FollowJointTrajectoryGoalConstPtr;

// Implementations for CS66FollowTrajectoryServer declared in header

CS66FollowTrajectoryServer::CS66FollowTrajectoryServer(ros::NodeHandle &nh, std::shared_ptr<CS66RobotController> controller)
  : nh_(nh),
    as_(nh_, "arm_controller/follow_joint_trajectory",
        boost::bind(&CS66FollowTrajectoryServer::executeCB, this, _1),
        false),
    robot_ctrl_(controller)
{
  sdk_joint_names_ = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
  as_.start();
  ROS_INFO("FollowJointTrajectory Action Server Started!");
}

CS66FollowTrajectoryServer::~CS66FollowTrajectoryServer(){
  std::cout << "CS66FollowTrajectoryServer destructor" << std::endl;
}

void CS66FollowTrajectoryServer::executeCB(const FollowGoalConstPtr& goal)
{
  control_msgs::FollowJointTrajectoryResult result;

  if (!goal) {
    result.error_code = result.INVALID_GOAL;
    return abortGoal("Empty goal");
  }

  const auto & traj = goal->trajectory;
  if (traj.joint_names.empty() || traj.points.empty()) {
    result.error_code = result.INVALID_GOAL;
    return abortGoal("Trajectory missing joint_names or points");
  }

  std::vector<int> idx_map = buildIndexMap(traj.joint_names);
  if (idx_map.empty()) {
    result.error_code = result.INVALID_JOINTS;
    return abortGoal("Joint mismatch");
  }

  std::vector<ELITE::vector6d_t> trajectory;
  std::vector<double> time_vec;
  trajectory.reserve(traj.points.size());
  time_vec.reserve(traj.points.size());
  double last_time_stamp = 0.0;

  for (const auto& pt : traj.points) {
    if (pt.positions.size() != traj.joint_names.size()) {
      result.error_code = result.INVALID_GOAL;
      return abortGoal("Trajectory point joint size mismatch");
    }

    ELITE::vector6d_t target_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (size_t j = 0; j < idx_map.size(); ++j) {
      if (idx_map[j] >= 0 && idx_map[j] < 6) {
        target_joints[idx_map[j]] = pt.positions[j];
      }
    }

    double current_time_stamp = pt.time_from_start.toSec(); 

    double duration = current_time_stamp - last_time_stamp;
    time_vec.push_back(duration);
    trajectory.push_back(target_joints);
    last_time_stamp = current_time_stamp;
  }

  ROS_INFO("Trajectory received: %zu points. Sending to SDK...", trajectory.size());

  if (as_.isPreemptRequested()) {
    control_msgs::FollowJointTrajectoryResult preempt_result;
    as_.setPreempted(preempt_result, "Preempt requested before execution");
    return;
  }

  try {
    robot_ctrl_->resetMotionFinish();

    if (!robot_ctrl_->RunServojTrajectory(trajectory, time_vec)) {
      result.error_code = result.PATH_TOLERANCE_VIOLATED;
      return abortGoal("SDK execution failed: RunServojTrajectory returned false");
    }
  }
  catch (std::exception& e) {
    result.error_code = result.PATH_TOLERANCE_VIOLATED;
    return abortGoal(std::string("SDK execution failed: ") + e.what());
  }

  result.error_code = result.SUCCESSFUL;
  as_.setSucceeded(result, "Trajectory completed");
  ROS_INFO("FollowJointTrajectory succeeded.");
}

std::vector<int> CS66FollowTrajectoryServer::buildIndexMap(const std::vector<std::string>& in_names)
{
  std::vector<int> idx_map(in_names.size(), -1);

  for (size_t i = 0; i < in_names.size(); ++i) {
    auto it = std::find(sdk_joint_names_.begin(), sdk_joint_names_.end(), in_names[i]);
    if (it == sdk_joint_names_.end())
      return {};

    idx_map[i] = std::distance(sdk_joint_names_.begin(), it);
  }
  return idx_map;
}

void CS66FollowTrajectoryServer::abortGoal(const std::string& msg)
{
  ROS_ERROR("%s", msg.c_str());
  control_msgs::FollowJointTrajectoryResult res;
  res.error_code = res.INVALID_GOAL;
  as_.setAborted(res, msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cs66_follow_joint_trajectory_server");
  ros::NodeHandle nh;
  // 创建 Controller 并传入 action server
  ros::NodeHandle private_nh("~");
  auto controller = std::make_shared<CS66RobotController>(nh, private_nh);
  CS66FollowTrajectoryServer server(nh, controller);
  ros::spin();
  return 0;
}