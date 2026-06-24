// ============================================================================
// camera_calib_eval_node.cpp
//
// Hand-eye calibration evaluation helper:
//   1. Move to an observation pose.
//   2. Read the chessboard pose from TF.
//   3. Verify the chessboard +Z axis points downward.
//   4. Move above the upper-left inner corner.
//   5. Slowly move in a straight line above the upper-right inner corner.
// ============================================================================

#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "cs66_follow_joint_traj_server.h"
#include "cs66_robot_controller.h"

namespace {

constexpr double kIkTimeoutSeconds = 0.2;
constexpr double kMinCartesianFraction = 0.9;
const char* kApproachPlannerId = "RRTstar";

struct EvalParams {
    std::string base_frame = "base_link";
    std::string ee_link = "ee_link";
    std::string board_frame = "chessboard_board";
    int inner_corners_x = 11;
    double square_size = 0.03;
    double hover_height = 0.08;
    double tf_timeout = 3.0;
    double cartesian_step = 0.005;
    double velocity_scaling = 0.03;
    double acceleration_scaling = 0.05;
    std::vector<double> observation_joints = {-0.63, -1.86, -1.18, -1.67, 1.57, -2.14};
    std::vector<double> home_joints = {0.04, -0.11, -2.7, -1.24, 1.45, -0.37};
};

void WaitForEnter(const std::string& prompt) {
    std::cout << prompt << std::endl;
    std::cin.get();
}

bool ReadDoubleVectorParam(
    const ros::NodeHandle& pnh,
    const std::string& name,
    std::vector<double>& values,
    size_t expected_size) {
    XmlRpc::XmlRpcValue raw;
    if (!pnh.getParam(name, raw)) {
        return true;
    }
    if (raw.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        static_cast<size_t>(raw.size()) != expected_size) {
        ROS_ERROR("~%s must be a list with %lu numeric values.",
                  name.c_str(), expected_size);
        return false;
    }

    std::vector<double> parsed;
    parsed.reserve(expected_size);
    for (int i = 0; i < raw.size(); ++i) {
        if (raw[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            parsed.push_back(static_cast<int>(raw[i]));
        } else if (raw[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            parsed.push_back(static_cast<double>(raw[i]));
        } else {
            ROS_ERROR("~%s[%d] must be numeric.", name.c_str(), i);
            return false;
        }
    }

    values = parsed;
    return true;
}

bool LoadParams(const ros::NodeHandle& pnh, EvalParams& params) {
    pnh.param<std::string>("base_frame", params.base_frame, params.base_frame);
    pnh.param<std::string>("ee_link", params.ee_link, params.ee_link);
    pnh.param<std::string>("board_frame", params.board_frame, params.board_frame);
    pnh.param<int>("inner_corners_x", params.inner_corners_x, params.inner_corners_x);
    pnh.param<double>("square_size", params.square_size, params.square_size);
    pnh.param<double>("hover_height", params.hover_height, params.hover_height);
    pnh.param<double>("tf_timeout", params.tf_timeout, params.tf_timeout);
    pnh.param<double>("cartesian_step", params.cartesian_step, params.cartesian_step);
    pnh.param<double>("velocity_scaling", params.velocity_scaling, params.velocity_scaling);
    pnh.param<double>("acceleration_scaling", params.acceleration_scaling, params.acceleration_scaling);

    if (!ReadDoubleVectorParam(pnh, "observation_joints", params.observation_joints, 6)) {
        return false;
    }
    if (!ReadDoubleVectorParam(pnh, "home_joints", params.home_joints, 6)) {
        return false;
    }

    if (params.inner_corners_x < 2) {
        ROS_ERROR("~inner_corners_x must be >= 2.");
        return false;
    }
    if (params.square_size <= 0.0 || params.hover_height <= 0.0 ||
        params.tf_timeout <= 0.0 || params.cartesian_step <= 0.0) {
        ROS_ERROR("square_size, hover_height, tf_timeout, and cartesian_step must be positive.");
        return false;
    }
    return true;
}

tf2::Vector3 Normalize(const tf2::Vector3& value) {
    tf2::Vector3 normalized = value;
    normalized.normalize();
    return normalized;
}

tf2::Quaternion QuaternionFromAxes(
    const tf2::Vector3& x_axis,
    const tf2::Vector3& y_axis,
    const tf2::Vector3& z_axis) {
    tf2::Matrix3x3 matrix(
        x_axis.x(), y_axis.x(), z_axis.x(),
        x_axis.y(), y_axis.y(), z_axis.y(),
        x_axis.z(), y_axis.z(), z_axis.z());
    tf2::Quaternion q;
    matrix.getRotation(q);
    q.normalize();
    return q;
}

geometry_msgs::Pose BuildPose(
    const tf2::Vector3& position,
    const tf2::Quaternion& orientation) {
    geometry_msgs::Pose pose;
    pose.position.x = position.x();
    pose.position.y = position.y();
    pose.position.z = position.z();
    pose.orientation = tf2::toMsg(orientation);
    return pose;
}

void PrintVector(const std::string& label, const tf2::Vector3& value) {
    ROS_INFO("%s: [%.6f, %.6f, %.6f]",
             label.c_str(), value.x(), value.y(), value.z());
}

void PrintPose(const std::string& label, const geometry_msgs::Pose& pose) {
    ROS_INFO("%s position: [%.6f, %.6f, %.6f]",
             label.c_str(), pose.position.x, pose.position.y, pose.position.z);
    ROS_INFO("%s quat: [%.6f, %.6f, %.6f, %.6f]",
             label.c_str(), pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w);
}

bool MoveToPoseWithOptimizingPlanner(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::Pose& target_pose) {
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    const moveit::core::JointModelGroup* joint_model_group =
        current_state->getJointModelGroup(move_group.getName());
    if (joint_model_group == nullptr) {
        ROS_ERROR("Failed to get joint model group: %s", move_group.getName().c_str());
        return false;
    }

    const bool found_ik = current_state->setFromIK(
        joint_model_group, target_pose, kIkTimeoutSeconds);
    if (!found_ik) {
        ROS_ERROR("IK failed for target pose. Consider keeping the current observation orientation.");
        PrintPose("IK failed target", target_pose);
        return false;
    }

    std::vector<double> target_joint_values;
    current_state->copyJointGroupPositions(joint_model_group, target_joint_values);

    move_group.setStartStateToCurrentState();
    move_group.setPlannerId(kApproachPlannerId);
    move_group.setJointValueTarget(target_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group.plan(plan);
    if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to plan approach motion with planner %s.", kApproachPlannerId);
        return false;
    }

    const auto exec_result = move_group.execute(plan);
    if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to execute approach motion.");
        return false;
    }

    return true;
}

bool MoveToJointTarget(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::vector<double>& joints,
    const std::string& description) {
    ROS_INFO("Moving to %s.", description.c_str());
    move_group.setStartStateToCurrentState();
    move_group.setJointValueTarget(joints);
    const auto result = move_group.move();
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to move to %s.", description.c_str());
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_calib_eval_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    EvalParams params;
    if (!LoadParams(pnh, params)) {
        return -1;
    }

    auto controller = std::make_shared<CS66RobotController>(nh, pnh);
    ros::Duration(1.0).sleep();
    CS66FollowTrajectoryServer moveit_server(nh, controller);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("arm_group");
    move_group.setPoseReferenceFrame(params.base_frame);
    move_group.setEndEffectorLink(params.ee_link);
    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(params.velocity_scaling);
    move_group.setMaxAccelerationScalingFactor(params.acceleration_scaling);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    ros::Duration(0.5).sleep();

    WaitForEnter("按Enter移动至相机标定评估观测点位...");
    if (!MoveToJointTarget(move_group, params.observation_joints, "observation pose")) {
        return -1;
    }
    ros::Duration(0.5).sleep();

    WaitForEnter("按Enter读取棋盘格TF并计算左右上角悬空点...");

    geometry_msgs::TransformStamped board_tf_msg;
    try {
        board_tf_msg = tf_buffer.lookupTransform(
            params.base_frame,
            params.board_frame,
            ros::Time(0),
            ros::Duration(params.tf_timeout));
    } catch (const tf2::TransformException& exc) {
        ROS_ERROR("Failed to lookup TF %s <- %s: %s",
                  params.base_frame.c_str(), params.board_frame.c_str(), exc.what());
        return -1;
    }

    tf2::Transform board_tf;
    tf2::fromMsg(board_tf_msg.transform, board_tf);
    const tf2::Vector3 board_origin = board_tf.getOrigin();
    const tf2::Matrix3x3 board_basis = board_tf.getBasis();
    const tf2::Vector3 board_x = Normalize(board_basis * tf2::Vector3(1.0, 0.0, 0.0));
    const tf2::Vector3 board_y = Normalize(board_basis * tf2::Vector3(0.0, 1.0, 0.0));
    const tf2::Vector3 board_z = Normalize(board_basis * tf2::Vector3(0.0, 0.0, 1.0));

    ROS_INFO("=== Chessboard TF Inspection ===");
    PrintVector("Board origin in base", board_origin);
    PrintVector("Board +X in base", board_x);
    PrintVector("Board +Y in base", board_y);
    PrintVector("Board +Z in base", board_z);

    if (board_z.z() > 0.0) {
        ROS_ERROR("Chessboard +Z points upward in base_link (z=%.6f). It must point downward.",
                  board_z.z());
        return -1;
    }

    const double right_x_offset = (params.inner_corners_x - 1) * params.square_size;
    const tf2::Vector3 left_corner = board_origin;
    const tf2::Vector3 right_corner = board_origin + board_x * right_x_offset;
    const tf2::Vector3 hover_offset = board_z * (-params.hover_height);
    const tf2::Vector3 left_hover = left_corner + hover_offset;
    const tf2::Vector3 right_hover = right_corner + hover_offset;
    const double corner_distance = (right_corner - left_corner).length();

    const tf2::Quaternion target_orientation = QuaternionFromAxes(board_x, board_y, board_z);
    const geometry_msgs::Pose left_pose = BuildPose(left_hover, target_orientation);
    const geometry_msgs::Pose right_pose = BuildPose(right_hover, target_orientation);

    PrintVector("Left upper inner corner", left_corner);
    PrintVector("Right upper inner corner", right_corner);
    PrintVector("Hover offset (-board_z * hover_height)", hover_offset);
    PrintPose("Left hover target", left_pose);
    PrintPose("Right hover target", right_pose);
    ROS_INFO("Left-right inner-corner distance: %.6f m", corner_distance);
    ROS_INFO("Expected distance from params: %.6f m", right_x_offset);
    ROS_INFO("Chessboard +Z is downward enough for evaluation (base z=%.6f).", board_z.z());

    WaitForEnter("按Enter移动到棋盘左上角内角点上方...");
    if (!MoveToPoseWithOptimizingPlanner(move_group, left_pose)) {
        return -1;
    }
    ros::Duration(0.5).sleep();

    ROS_INFO("Planning Cartesian line from left hover target to right hover target.");
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(right_pose);

    moveit_msgs::RobotTrajectory trajectory;
    const double fraction = move_group.computeCartesianPath(
        waypoints, params.cartesian_step, trajectory);
    ROS_INFO("Cartesian path fraction: %.2f%%", fraction * 100.0);
    if (fraction < kMinCartesianFraction) {
        ROS_ERROR("Cartesian path fraction is below %.2f%%. Not executing.",
                  kMinCartesianFraction * 100.0);
        return -1;
    }

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;

    WaitForEnter("按Enter沿棋盘上边缘从左上角上方缓慢移动到右上角上方...");
    const auto cartesian_exec_result = move_group.execute(cartesian_plan);
    if (cartesian_exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to execute Cartesian evaluation path.");
        return -1;
    }
    ROS_INFO("Calibration evaluation line motion finished.");

    WaitForEnter("按Enter返回收纳姿态...");
    if (!MoveToJointTarget(move_group, params.home_joints, "home pose")) {
        return -1;
    }

    ros::waitForShutdown();
    return 0;
}
