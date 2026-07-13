// ============================================================================
// task_main.cpp — 钢印打磨任务主控节点
//
// 整体角色：感知（OCR）与执行（机械臂）之间的桥梁。
//   1. 控制机械臂运动到观测姿态
//   2. 调用 OCR 服务获取钢印轨迹
//   3. 运动到轨迹起点
//   4. 沿轨迹执行笛卡尔空间运动（配合 Z 方向力控打磨）
//   5. 返回安全位置
//
// 硬件平台：艾利特 EC66 协作机器人 + MoveIt 运动规划
// ============================================================================

#include <cmath>
#include <atomic>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>  // MoveIt C++ API
#include <moveit/robot_state/robot_state.h>                    // 机器人状态（IK 用）
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>                         // 四元数 → 轴角转换
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>               // TF2 与 ROS 消息互转

#include "cs66_robot_controller.h"          // 艾利特 EC66 底层控制器封装（含力控接口）
#include "cs66_follow_joint_traj_server.h"  // 关节轨迹跟随服务
#include "car_ctl/CarAutoTaskAction.h"      // 小车横移自动任务 action
#include "mv3d_rgbd_ros/DetectSteelStamp.h" // OCR 服务定义（自动生成的头文件）

namespace {

// ---------------------------------------------------------------------------
// 全局常量
// ---------------------------------------------------------------------------

// 笛卡尔路径插值步长（米）：MoveIt 在相邻路点之间每 5mm 插一个点
constexpr double kCartesianStepMeters = 0.005;

// 每轮扫查结束后，小车横向移动距离默认值（米），可通过 ~car_move_distance_meters 覆盖。
constexpr double kDefaultCarMoveDistanceMeters = -0.9;

// 第一个接近点使用的规划器：RRTstar（渐进最优的采样规划器，路径质量好但稍慢）
const char* kFirstApproachPlannerId = "RRTstar";

std::atomic<bool> g_shutdown_requested(false);

// ===========================================================================
// 辅助函数
// ===========================================================================

void SignalHandler(int) {
    g_shutdown_requested.store(true);
}

bool IsShutdownRequested() {
    return !ros::ok() || g_shutdown_requested.load();
}

bool WaitForOperatorEnter(const std::string& prompt) {
    if (IsShutdownRequested()) {
        return false;
    }

    std::cout << prompt << std::endl;
    std::string input;
    if (!std::getline(std::cin, input)) {
        return false;
    }
    return !IsShutdownRequested();
}

// ---------------------------------------------------------------------------
// ConvertTrajectoryToSDK
//   将 MoveIt 的 RobotTrajectory 转换为艾利特 SDK 的关节角数组格式。
//   MoveIt 轨迹: 每个点包含 6 个关节角 + 时间戳
//   艾利特 SDK:  vector6d_t（6 个 double 的数组）+ 时间数组
//
//   注意：当前代码中此函数被注释掉（改用 MoveIt 直接执行），
//   保留作为备选方案——如果需要绕过 MoveIt 直接用 SDK 的 servoj 执行。
// ---------------------------------------------------------------------------
std::vector<ELITE::vector6d_t> ConvertTrajectoryToSDK(
    const moveit_msgs::RobotTrajectory& plan_traj,
    std::vector<double>& times) {
    std::vector<ELITE::vector6d_t> sdk_traj;
    const auto& points = plan_traj.joint_trajectory.points;

    sdk_traj.reserve(points.size());
    times.reserve(points.size());
    for (const auto& point : points) {
        ELITE::vector6d_t joints;
        for (size_t i = 0; i < 6 && i < point.positions.size(); ++i) {
            joints[i] = point.positions[i];              // 拷贝关节角
        }
        sdk_traj.push_back(joints);
        times.push_back(point.time_from_start.toSec());  // 记录时间戳
    }
    return sdk_traj;
}

// ---------------------------------------------------------------------------
// PrintTargetPoses
//   打印 OCR 返回的所有目标姿态，供操作员在终端检查。
//   中文提示提醒操作员先去 debug_images 目录确认 OCR 识别质量。
// ---------------------------------------------------------------------------
void PrintTargetPoses(const std::vector<geometry_msgs::Pose>& target_poses) {
    ROS_INFO("=== Debugging Target Poses（先去debug_images里看看识别的是否准确） ===");
    for (size_t i = 0; i < target_poses.size(); ++i) {
        const auto& pose = target_poses[i];
        ROS_INFO("Pose[%lu] Position: [x: %.4f, y: %.4f, z: %.4f]",
                 i, pose.position.x, pose.position.y, pose.position.z);
        ROS_INFO("Pose[%lu] Quat: [x: %.4f, y: %.4f, z: %.4f, w: %.4f]",
                 i, pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w);
        ROS_INFO("---------------------------------------------");
    }
}

bool IsRetryInput(const std::string& input) {
    return input == "r" || input == "R";
}

// ---------------------------------------------------------------------------
// MoveToPoseWithOptimizingPlanner
//   使用"手动 IK + 关节空间规划"的方式运动到目标姿态。
//
//   为什么不用 MoveIt 默认的 setPoseTarget？
//     默认 setPoseTarget 在笛卡尔空间规划，可能产生大幅关节转动。
//     这里先手动调 IK（以当前关节角为种子找最近解），
//     然后用 RRTstar 在关节空间规划——路径更可控、关节跳变更小。
//
//   流程：
//     1. 获取当前机器人状态
//     2. setFromIK: 从目标姿态反解关节角（以当前关节角为种子）
//     3. 将 IK 解作为关节空间目标
//     4. RRTstar 规划 + 执行
//
//   参数:
//     move_group:  MoveIt 运动组接口
//     target_pose: 目标末端姿态（相对于 base_link）
//   返回:
//     true=成功到达, false=IK/规划/执行失败
// ---------------------------------------------------------------------------
bool MoveToPoseWithOptimizingPlanner(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::Pose& target_pose) {

    // 1. 获取当前机器人状态和关节模型组
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    const moveit::core::JointModelGroup* joint_model_group =
        current_state->getJointModelGroup(move_group.getName());
    std::vector<double> current_joint_values;
    current_state->copyJointGroupPositions(joint_model_group, current_joint_values);

    const std::vector<double> preferred_seed_joints = {
        0.03, -2.71, -2.43, 0.45, 1.57, -1.55
    };
    if (preferred_seed_joints.size() == current_joint_values.size()) {
        current_state->setJointGroupPositions(joint_model_group, preferred_seed_joints);
        ROS_INFO("[DEBUG][FirstIK] Using preferred seed joints for first approach IK.");
    } else {
        ROS_WARN("[DEBUG][FirstIK] preferred_seed_joints size=%zu, expected=%zu. Using current state as IK seed.",
                 preferred_seed_joints.size(), current_joint_values.size());
    }

    // 2. 逆运动学求解
    //    setFromIK(target_pose, timeout):
    //    - 以 current_state 中的当前关节角作为种子点
    //    - 在 0.1 秒内寻找最近的有效 IK 解
    //    - 成功则 current_state 中的关节值被更新为 IK 解
    //    这样做的好处：IK 解与当前姿态关节角差异最小，避免大幅跳变
    bool found_ik = current_state->setFromIK(joint_model_group, target_pose, 0.1);
    if (!found_ik) {
        ROS_ERROR("IK Solver failed to find a valid joint solution for the first approach point.");
        return false;
    }

    // 3. 提取 IK 解出的目标关节角
    std::vector<double> target_joint_values;
    current_state->copyJointGroupPositions(joint_model_group, target_joint_values);
    for (size_t i = 0; i < target_joint_values.size(); ++i) {
        const double current_delta =
            i < current_joint_values.size() ? target_joint_values[i] - current_joint_values[i] : 0.0;
        const double seed_delta =
            i < preferred_seed_joints.size() ? target_joint_values[i] - preferred_seed_joints[i] : 0.0;
        ROS_INFO("[DEBUG][FirstIK] joint[%zu] current=%.6f seed=%.6f ik=%.6f ik-current=%.6f ik-seed=%.6f",
                 i,
                 i < current_joint_values.size() ? current_joint_values[i] : 0.0,
                 i < preferred_seed_joints.size() ? preferred_seed_joints[i] : 0.0,
                 target_joint_values[i],
                 current_delta,
                 seed_delta);
    }
    // [安全提示]：可在此处打印 target_joint_values，
    // 与当前关节角对比，确保 J1/J2 变化量 < 0.5 rad

    // 4. 关节空间规划（而非笛卡尔空间）
    move_group.setStartStateToCurrentState();            // 从当前状态出发
    move_group.setPlannerId(kFirstApproachPlannerId);    // 使用 RRTstar
    move_group.setJointValueTarget(target_joint_values); // 关节角目标（关键！不用 PoseTarget）

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group.plan(plan);
    if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to plan motion to the first approach point with planner %s.",
                  kFirstApproachPlannerId);
        move_group.clearPoseTargets();
        return false;
    }

    // 5. 执行规划结果
    const auto exec_result = move_group.execute(plan);
    if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to execute motion to the first approach point.");
        move_group.clearPoseTargets();
        return false;
    }

    move_group.clearPoseTargets();
    return true;
}

// ---------------------------------------------------------------------------
// BuildTrackingWaypoints
//   从 OCR 返回的轨迹中构建笛卡尔路径的路点列表。
//
//   策略：丢弃第一个点（因为 Step 3 已经单独运动到了起点），
//   从第二个点开始作为笛卡尔路径的路点。
//   如果只有 1 个点，则直接返回该点（原地不动）。
//
//   为什么丢弃第一个点？
//     Step 3 已经用关节空间规划精确到达了起点，
//     笛卡尔路径不需要再包含起点，避免冗余运动和潜在的起点跳变。
// ---------------------------------------------------------------------------
std::vector<geometry_msgs::Pose> BuildTrackingWaypoints(
    const std::vector<geometry_msgs::Pose>& target_poses) {
    std::vector<geometry_msgs::Pose> waypoints;
    if (target_poses.empty()) {
        return waypoints;
    }
    waypoints.reserve(target_poses.size() > 1 ? target_poses.size() - 1 : 1);
    // 从 index=1 开始，跳过第一个点
    for (size_t i = 1; i < target_poses.size(); ++i) {
        waypoints.push_back(target_poses[i]);
    }
    // 防御：如果只有一个点，至少返回它
    if (waypoints.empty()) {
        waypoints.push_back(target_poses.front());
    }
    return waypoints;
}

// ---------------------------------------------------------------------------
// PrintTrajectoryDebug
//   打印 MoveIt 规划的笛卡尔轨迹的详细调试信息。
//
//   输出内容：
//     1. 轨迹点总数
//     2. 每个点的关节角-时间表（表格形式）
//     3. 轨迹起点 vs 当前实际关节角对比
//        - 如果某关节差值 > 0.1 rad，标记 "JUMP DETECTED!"
//        - 跳变通常意味着 IK 选择了不连续的解，可能导致危险运动
// ---------------------------------------------------------------------------
void PrintTrajectoryDebug(
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::RobotTrajectory& trajectory) {

    std::cout << "\n================ [DEBUG] Trajectory Inspection ================" << std::endl;
    const auto& joint_names = trajectory.joint_trajectory.joint_names;
    const auto& points = trajectory.joint_trajectory.points;

    // 打印表头
    std::cout << "Planned Points Count: " << points.size() << std::endl;
    std::cout << "Index | Time   | ";
    for (const auto& name : joint_names) {
        std::cout << std::setw(10) << name << " | ";
    }
    std::cout << std::endl;

    // 逐点打印关节角和时间
    for (size_t i = 0; i < points.size(); ++i) {
        std::cout << std::setw(5) << i << " | "
                  << std::fixed << std::setprecision(3)
                  << points[i].time_from_start.toSec() << "s | ";
        for (double value : points[i].positions) {
            std::cout << std::setw(10) << std::setprecision(4) << value << " | ";
        }
        std::cout << std::endl;
    }

    // 起点跳变检查：轨迹第一个规划点 vs 当前实际关节角
    std::cout << "\n================ [DEBUG] Start Point Check ================" << std::endl;
    std::vector<double> current_joints = move_group.getCurrentJointValues();
    if (!points.empty()) {
        std::cout << "Joint ID | Current (Real) | Trajectory[0] (Plan) | Diff (Rad)" << std::endl;
        for (size_t j = 0; j < current_joints.size(); ++j) {
            const double diff = points[0].positions[j] - current_joints[j];
            printf("Joint %zu  | %14.6f | %18.6f | %10.6f",
                   j, current_joints[j], points[0].positions[j], diff);
            if (std::abs(diff) > 0.1) {                  // 0.1 rad ≈ 5.7° 阈值
                std::cout << " <--- JUMP DETECTED!";      // 警告：关节跳变！
            }
            std::cout << std::endl;
        }
    }
    std::cout << "===========================================================\n" << std::endl;
}

bool MoveToJointTarget(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::vector<double>& target_joints,
    const std::string& prompt) {

    if (!WaitForOperatorEnter(prompt)) {
        return false;
    }

    move_group.setStartStateToCurrentState();
    move_group.setJointValueTarget(target_joints);
    const auto move_result = move_group.move();
    if (move_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("MoveIt failed to move to requested joint target.");
        return false;
    }
    return !IsShutdownRequested();
}

bool RunOcrWithRetry(
    ros::ServiceClient& ocr_client,
    std::vector<geometry_msgs::Pose>& target_poses) {

    target_poses.clear();
    while (!IsShutdownRequested()) {
        std::cout << "按Enter启动OCR detection，或输入R后回车重新执行OCR..." << std::endl;
        std::string input;
        if (!std::getline(std::cin, input)) {
            return false;
        }

        ROS_INFO("Step 2: OCR Detection");
        mv3d_rgbd_ros::DetectSteelStamp srv;

        if (!ocr_client.call(srv)) {
            ROS_ERROR("Failed to call OCR service");
            std::cout << "OCR服务调用失败。输入R后回车重试，其他输入退出任务: ";
            if (!std::getline(std::cin, input)) {
                return false;
            }
            if (IsRetryInput(input)) {
                continue;
            }
            return false;
        }

        if (!srv.response.success) {
            ROS_ERROR("OCR service returned failure: %s", srv.response.message.c_str());
            std::cout << "OCR返回失败。输入R后回车重试，其他输入退出任务: ";
            if (!std::getline(std::cin, input)) {
                return false;
            }
            if (IsRetryInput(input)) {
                continue;
            }
            return false;
        }

        if (srv.response.poses.size() < 2) {
            ROS_ERROR("OCR service returned too few target poses: %lu", srv.response.poses.size());
            std::cout << "OCR点位数量不足。输入R后回车重试，其他输入退出任务: ";
            if (!std::getline(std::cin, input)) {
                return false;
            }
            if (IsRetryInput(input)) {
                continue;
            }
            return false;
        }

        target_poses = srv.response.poses;
        ROS_INFO("Received %lu poses from OCR node.", target_poses.size());
        PrintTargetPoses(target_poses);

        std::cout << "请检查debug_images/RViz中的OCR结果。按Enter接受当前结果，输入R后回车重新执行OCR: ";
        if (!std::getline(std::cin, input)) {
            return false;
        }
        if (IsRetryInput(input)) {
            continue;
        }
        return true;
    }

    return false;
}

bool RunCarAutoTask(
    actionlib::SimpleActionClient<car_ctl::CarAutoTaskAction>& car_client,
    double distance_meters) {

    ROS_INFO("Waiting for car_auto_task action server...");
    while (!IsShutdownRequested()) {
        if (car_client.waitForServer(ros::Duration(0.5))) {
            break;
        }
    }
    if (IsShutdownRequested()) {
        return false;
    }

    car_ctl::CarAutoTaskGoal goal;
    goal.distance = static_cast<float>(distance_meters);

    ROS_INFO("Sending car_auto_task goal: distance=%.3f m", distance_meters);
    car_client.sendGoal(goal);

    ros::Rate rate(10);
    while (ros::ok() && !car_client.getState().isDone()) {
        if (g_shutdown_requested.load()) {
            ROS_WARN("Stop requested. Cancelling car_auto_task goal before exit...");
            car_client.cancelAllGoals();
            car_client.waitForResult(ros::Duration(2.0));
            ros::shutdown();
            return false;
        }
        rate.sleep();
    }

    const actionlib::SimpleClientGoalState state = car_client.getState();
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_ERROR("car_auto_task failed: %s", state.toString().c_str());
        return false;
    }

    const car_ctl::CarAutoTaskResultConstPtr result = car_client.getResult();
    if (!result || !result->success) {
        ROS_ERROR("car_auto_task result is not successful: %s",
                  result ? result->message.c_str() : "no result");
        return false;
    }

    ROS_INFO("car_auto_task succeeded: %s", result->message.c_str());
    return true;
}

AdmittanceForceControlParams LoadAdmittanceParams(const ros::NodeHandle& pnh) {
    AdmittanceForceControlParams params;
    pnh.param("target_force_z", params.target_force_z, params.target_force_z);
    pnh.param("force_sign", params.force_sign, params.force_sign);
    pnh.param("admittance_mass", params.admittance_mass, params.admittance_mass);
    pnh.param("admittance_damping", params.admittance_damping, params.admittance_damping);
    pnh.param("admittance_stiffness", params.admittance_stiffness, params.admittance_stiffness);
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
    return params;
}

bool RunArmScanCycle(
    ros::NodeHandle& nh,
    ros::ServiceClient& ocr_client,
    const std::shared_ptr<CS66RobotController>& my_controller,
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::vector<double>& observation_joints,
    const std::vector<double>& shouna_joints,
    const AdmittanceForceControlParams& admittance_params) {

    // =======================================================================
    // Step 1: 运动到观测点位
    // =======================================================================
    ROS_INFO("Step 1: Go observation pose");
    if (!MoveToJointTarget(move_group, observation_joints, "按Enter继续移动至观测点位...")) {
        return false;
    }

    // =======================================================================
    // Step 2: OCR 检测
    // =======================================================================
    std::vector<geometry_msgs::Pose> target_poses;
    if (!RunOcrWithRetry(ocr_client, target_poses)) {
        return false;
    }

    // =======================================================================
    // Step 3: 运动到轨迹的第一个点
    // =======================================================================
    ROS_INFO("Step 3: Move to the first pose returned by OCR trajectory");
    geometry_msgs::Pose first_target = target_poses.front();

    if (!WaitForOperatorEnter("按Enter移动到第一个点法向正上方的位置...")) {
        return false;
    }
    if (!MoveToPoseWithOptimizingPlanner(move_group, first_target)) {
        return false;
    }

    ros::Duration(0.5).sleep();

    // =======================================================================
    // Step 4: 笛卡尔轨迹 + 力控打磨执行
    // =======================================================================
    ROS_INFO("Step 4: Build cartesian tracking trajectory through detected points");

    std::vector<geometry_msgs::Pose> waypoints = BuildTrackingWaypoints(target_poses);

    // 从新计算起始点，避免moveit判断当前点与起始不同而execute失败
    ros::Duration(0.5).sleep();
    move_group.setStartStateToCurrentState();

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(
        waypoints, kCartesianStepMeters, trajectory);

    PrintTrajectoryDebug(move_group, trajectory);

    if (fraction < 0.9) {
        ROS_WARN("Cartesian path incomplete (%.2f%%).", fraction * 100.0);
    }

    if (trajectory.joint_trajectory.points.empty()) {
        ROS_ERROR("Refusing to run admittance control with empty nominal trajectory.");
        return false;
    }

    if (!WaitForOperatorEnter("开始执行手动导纳力控轨迹...")) {
        return false;
    }

    if (!my_controller->RunJointServoAdmittanceTrajectory(target_poses, move_group, admittance_params)) {
        ROS_ERROR("Manual admittance trajectory execution failed!");
        return false;
    }

    ROS_INFO("Manual admittance trajectory finished.");

    ros::Duration(5).sleep();

    // =======================================================================
    // Step 5: 返回安全位置（收納姿态）
    // =======================================================================
    ROS_INFO("Step 5: Return to safe stow pose");
    if (!MoveToJointTarget(move_group, shouna_joints, "按Enter回原点...")) {
        return false;
    }

    return true;
}

}  // namespace



// ===========================================================================
// main — 循环扫查主流程
// ===========================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "steel_task_node", ros::init_options::NoSigintHandler);
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    const double car_move_distance_meters =
        pnh.param<double>("car_move_distance_meters", kDefaultCarMoveDistanceMeters);
    ROS_INFO("Car move distance per scan cycle: %.3f m", car_move_distance_meters);

    auto my_controller = std::make_shared<CS66RobotController>(nh, pnh);
    ros::Duration(1.0).sleep();

    CS66FollowTrajectoryServer moveit_server(nh, my_controller);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("arm_group");
    move_group.setPoseReferenceFrame("base_link");
    move_group.setEndEffectorLink("ee_link");
    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.1);

    const std::vector<double> observation_joints = {0.03, -0.93, -2.34, -1.44, 1.57, -1.55};
    const std::vector<double> shouna_joints = {0.04, -0.11, -2.7, -1.24, 1.45, -0.37};

    ros::ServiceClient ocr_client =
        nh.serviceClient<mv3d_rgbd_ros::DetectSteelStamp>("get_steel_stamp_location");

    actionlib::SimpleActionClient<car_ctl::CarAutoTaskAction> car_client("car_auto_task", true);
    const AdmittanceForceControlParams admittance_params = LoadAdmittanceParams(pnh);
    ROS_INFO("Admittance params: target_fz=%.3f sign=%.1f M=%.3f B=%.3f K=%.3f dt=%.4f speed=%.4f",
             admittance_params.target_force_z,
             admittance_params.force_sign,
             admittance_params.admittance_mass,
             admittance_params.admittance_damping,
             admittance_params.admittance_stiffness,
             admittance_params.control_period,
             admittance_params.nominal_tcp_speed);

    int cycle_index = 1;
    while (!IsShutdownRequested()) {
        ROS_INFO("========== Start arm scan cycle %d ==========", cycle_index);

        if (!RunArmScanCycle(
                nh, ocr_client, my_controller, move_group, observation_joints, shouna_joints,
                admittance_params)) {
            return IsShutdownRequested() ? 0 : -1;
        }

        std::cout << "当前轮次已回到安全位置。按Enter控制小车横移"
                  << car_move_distance_meters
                  << "m并进入下一轮；输入R后回车退出任务: ";
        std::string input;
        if (!std::getline(std::cin, input)) {
            return -1;
        }
        if (IsRetryInput(input)) {
            ROS_INFO("Operator requested exit after cycle %d.", cycle_index);
            return 0;
        }

        if (!RunCarAutoTask(car_client, car_move_distance_meters)) {
            return IsShutdownRequested() ? 0 : -1;
        }

        ++cycle_index;
    }

    return 0;
}
