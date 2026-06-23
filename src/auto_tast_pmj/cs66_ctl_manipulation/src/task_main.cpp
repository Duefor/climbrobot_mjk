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
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>  // MoveIt C++ API
#include <moveit/robot_state/robot_state.h>                    // 机器人状态（IK 用）
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>                         // 四元数 → 轴角转换
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>               // TF2 与 ROS 消息互转

#include "cs66_robot_controller.h"          // 艾利特 EC66 底层控制器封装（含力控接口）
#include "cs66_follow_joint_traj_server.h"  // 关节轨迹跟随服务
#include "mv3d_rgbd_ros/DetectSteelStamp.h" // OCR 服务定义（自动生成的头文件）

namespace {

// ---------------------------------------------------------------------------
// 全局常量
// ---------------------------------------------------------------------------

// 笛卡尔路径插值步长（米）：MoveIt 在相邻路点之间每 5mm 插一个点
constexpr double kCartesianStepMeters = 0.005;

// 第一个接近点使用的规划器：RRTstar（渐进最优的采样规划器，路径质量好但稍慢）
const char* kFirstApproachPlannerId = "RRTstar";

// ===========================================================================
// 辅助函数
// ===========================================================================

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

}  // namespace

// ===========================================================================
// main — 钢印打磨任务主流程
// ===========================================================================
int main(int argc, char** argv) {
    // -----------------------------------------------------------------------
    // 初始化 ROS
    // -----------------------------------------------------------------------
    ros::init(argc, argv, "steel_task_node");
    ros::NodeHandle nh;       // 全局命名空间句柄
    ros::NodeHandle pnh("~"); // 私有命名空间句柄（用于 ~param 参数）

    // -----------------------------------------------------------------------
    // 初始化机械臂控制器
    //   CS66RobotController: 封装了艾利特 EC66 的底层通信与控制
    //   （包含力控模式的启动/停止、关节伺服等）
    // -----------------------------------------------------------------------
    auto my_controller = std::make_shared<CS66RobotController>(nh, pnh);

    // 等待 1 秒让控制器完全就绪
    ros::Duration(1.0).sleep();

    // 启动关节轨迹跟随服务（MoveIt 执行轨迹时需要）
    CS66FollowTrajectoryServer moveit_server(nh, my_controller);

    // -----------------------------------------------------------------------
    // 启动异步 ROS 线程池
    //   2 个线程：1 个处理回调，1 个留给 MoveIt 内部使用
    // -----------------------------------------------------------------------
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // -----------------------------------------------------------------------
    // 配置 MoveIt 运动组
    // -----------------------------------------------------------------------
    moveit::planning_interface::MoveGroupInterface move_group("arm_group");

    // 所有目标姿态的参考坐标系 = 机器人基座
    move_group.setPoseReferenceFrame("base_link");

    // 末端执行器连杆名称（TCP 所在连杆）
    move_group.setEndEffectorLink("ee_link");

    // 规划参数：每次规划最多 5 秒、最多尝试 10 次
    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(10);

    // 速度与加速度限制（极保守——适用于精细打磨作业）
    //   5%  最大速度 ≈ 关节限速的 1/20
    //   10% 最大加速度
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.1);

    // -----------------------------------------------------------------------
    // 预设关节姿态（针对具体工位调试确定）
    // -----------------------------------------------------------------------

    // observation_joints: 观测姿态——机械臂摆好，相机对准钢印区域
    // 多组被注释的历史值记录了对不同工位/批次调试的微调过程
    // const std::vector<double> observation_joints = {0.85,-1.66,-1.62,-1.43,1.57,-0.72};
    // const std::vector<double> observation_joints = {0.87, -1.51, -1.44, -1.79, 1.56, -0.7};
    // const std::vector<double> observation_joints = {0.12, -1.38,-2.24,-1.09,1.57,-1.45};
    const std::vector<double> observation_joints = {0.12, -1.42, -1.94, -1.26, 1.57, -1.44};

    // shouna_joints: 收納姿态——任务结束后机械臂收回的安全位置
    const std::vector<double> shouna_joints = {0.04, -0.11, -2.7, -1.24, 1.45, -0.37};

    // =======================================================================
    // Step 1: 运动到观测点位
    //   操作员确认安全后按 Enter，机械臂从当前位置关节空间运动到观测姿态
    // =======================================================================
    ROS_INFO("Step 1: Go observation pose");
    std::cout << "按Enter继续移动至观测点位..." << std::endl;
    std::cin.get();                                       // 阻塞等待操作员确认
    move_group.setStartStateToCurrentState();              // 从当前状态出发
    move_group.setJointValueTarget(observation_joints);    // 关节角目标
    move_group.move();                                    // 规划+执行（阻塞）

    // =======================================================================
    // [已注释] 独立力控测试代码
    //   历史调试用途：不经过 OCR，直接开启 Z 方向力控 + 保持 15 秒
    // =======================================================================
    // std::cout << "按Enter开始力控轨迹执行..." << std::endl;
    // std::cin.get();
    // double target_force_z = -10.0;
    // if (!my_controller->startForceMode(target_force_z)) { ... }
    // ros::Duration(15).sleep();
    // my_controller->endForceMode();

    // =======================================================================
    // Step 2: OCR 检测——调用 Python 服务节点获取钢印轨迹
    // =======================================================================
    std::cout << "按Enter启动OCR detection..." << std::endl;
    std::cin.get();

    ROS_INFO("Step 2: OCR Detection");

    // 创建 ROS 服务客户端，连接到 OCR 服务
    //   服务名: get_steel_stamp_location
    //   服务类型: mv3d_rgbd_ros::DetectSteelStamp（空请求 → {success, message, poses[], texts[]}）
    ros::ServiceClient ocr_client =
        nh.serviceClient<mv3d_rgbd_ros::DetectSteelStamp>("get_steel_stamp_location");
    mv3d_rgbd_ros::DetectSteelStamp srv;

    // 同步调用（阻塞直到服务响应）
    if (!ocr_client.call(srv)) {
        ROS_ERROR("Failed to call OCR service");
        return -1;
    }
    if (!srv.response.success) {
        ROS_ERROR("OCR service returned failure: %s", srv.response.message.c_str());
        return -1;
    }

    // 获取 OCR 返回的目标姿态列表
    //   这些姿态由 Python 节点在 base_link 坐标系下计算
    //   可能经过了 B 样条插值 / 圆弧密度插值
    const std::vector<geometry_msgs::Pose> target_poses = srv.response.poses;
    if (target_poses.size() < 2) {
        ROS_ERROR("OCR service returned too few target poses: %lu", target_poses.size());
        return -1;
    }
    ROS_INFO("Received %lu poses from OCR node.", target_poses.size());

    // 打印所有目标姿态到终端，供操作员检查
    PrintTargetPoses(target_poses);

    // =======================================================================
    // Step 3: 运动到轨迹的第一个点
    //   使用 "手动 IK + RRTstar 关节空间规划" 精确到达起点
    // =======================================================================
    ROS_INFO("Step 3: Move to the first pose returned by OCR trajectory");
    geometry_msgs::Pose first_target = target_poses.front();  // 取第一个姿态

    std::cout << "按Enter移动到第一个点法向正上方的位置..." << std::endl;
    std::cin.get();
    if (!MoveToPoseWithOptimizingPlanner(move_group, first_target)) {
        return -1;
    }

    // 等待机械臂稳定
    ros::Duration(0.5).sleep();

    // =======================================================================
    // Step 4: 笛卡尔轨迹 + 力控打磨执行
    //   核心执行段。两个关键组件：
    //     (A) 笛卡尔路径: MoveIt computeCartesianPath 在直线段间插值
    //     (B) Z 方向力控: 保持 8N 恒力接触（补偿表面不平整）
    // =======================================================================
    ROS_INFO("Step 4: Build cartesian tracking trajectory through detected points");

    // --- 4A. 构建笛卡尔路点 ---
    // 丢弃第一个点（Step 3 已到达），从 index=1 开始
    std::vector<geometry_msgs::Pose> waypoints = BuildTrackingWaypoints(target_poses);

    // --- 4B. 笛卡尔路径规划 ---
    // computeCartesianPath:
    //   - 在相邻路点之间沿直线插值
    //   - 每个中间点调用 IK，保持末端沿直线运动
    //   - kCartesianStepMeters=0.005: 每隔 5mm 插一个点
    //   - fraction: 成功规划的比例（1.0=100%成功，<0.9 时发出警告）
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(
        waypoints, kCartesianStepMeters, trajectory);

    // 打印轨迹调试信息（关节角表 + 起点跳变检查）
    PrintTrajectoryDebug(move_group, trajectory);

    if (fraction < 0.9) {
        ROS_WARN("Cartesian path incomplete (%.2f%%).", fraction * 100.0);
    }

    // 将规划的轨迹包装为 MoveIt Plan
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;

    // ===================================================================
    // 4C. 力控参数配置
    //
    // 力控坐标系 (force_ref_frame):
    //   以当前 TCP 位姿为参考原点。
    //   force_ref_frame[0~2] = TCP 位置 (x, y, z)
    //   force_ref_frame[3~5] = TCP 姿态的四元数 → 轴角表示 (rx, ry, rz)
    //
    // 为什么需要 force_ref_frame？
    //   力控需要一个参考坐标系来描述"哪个方向是 Z"。
    //   这里直接读 /tcp_pose topic（由机器人驱动实时发布），
    //   用当前 TCP 位姿作为力控参考——Z 轴 = 工具末端轴向。
    // ===================================================================

    // 从 /tcp_pose topic 读取当前 TCP 位姿（等待最多 1 秒）
    auto tcp_pose_msg = ros::topic::waitForMessage<geometry_msgs::Pose>(
        "/tcp_pose", nh, ros::Duration(1.0));
    if (!tcp_pose_msg) {
        ROS_ERROR("Failed to read /tcp_pose for force_ref_frame.");
        return -1;
    }

    // 力控参考坐标系（艾利特 SDK 格式）
    //   前 3 个元素 [0~2] = 参考原点位置 (x, y, z)
    //   后 3 个元素 [3~5] = 参考姿态的轴角表示 (rx, ry, rz)
    ELITE::vector6d_t force_ref_frame;

    // 力控参考坐标系原点 = 当前 TCP 位置
    force_ref_frame[0] = tcp_pose_msg->position.x;
    force_ref_frame[1] = tcp_pose_msg->position.y;
    force_ref_frame[2] = tcp_pose_msg->position.z;

    // ---- 四元数 → 轴角 (axis-angle) 转换 ----
    // 艾利特 SDK 使用轴角表示姿态（而非四元数），需要手动转换
    //
    // 轴角公式:
    //   angle = 2 * acos(w)                   ← 旋转角度
    //   axis  = (x, y, z) / sin(angle/2)     ← 旋转轴（归一化后）
    //   轴角向量 = axis * angle               ← 方向和大小合一
    //
    // 为什么用轴角而非四元数？
    //   艾利特 SDK 的力控接口设计为轴角格式，
    //   这样力方向的计算更直观：力控参考系的 Z 轴方向直接由轴角决定
    tf2::Quaternion q(
        tcp_pose_msg->orientation.x,
        tcp_pose_msg->orientation.y,
        tcp_pose_msg->orientation.z,
        tcp_pose_msg->orientation.w);
    q.normalize();  // 归一化，防止浮点误差导致 |q| ≠ 1

    // 计算旋转角度：angle = 2 * acos(w)
    // 先钳制 w 到 [-1, 1] 防止 acos 域错误
    double w = q.w();
    if (w > 1.0) {
        w = 1.0;
    } else if (w < -1.0) {
        w = -1.0;
    }
    const double angle = 2.0 * std::acos(w);

    // 计算旋转轴：sin(angle/2) = sqrt(1 - w²)
    // s 是轴角公式的分母
    const double s = std::sqrt(std::max(0.0, 1.0 - w * w));
    if (s < 1e-8) {
        // s ≈ 0 意味着角度 ≈ 0（无旋转），轴角为零向量
        force_ref_frame[3] = 0.0;
        force_ref_frame[4] = 0.0;
        force_ref_frame[5] = 0.0;
    } else {
        // axis_x = q.x / sin(angle/2), 然后 rx = axis_x * angle
        const double axis_x = q.x() / s;
        const double axis_y = q.y() / s;
        const double axis_z = q.z() / s;
        force_ref_frame[3] = axis_x * angle;  // rx: 绕 X 轴旋转角度
        force_ref_frame[4] = axis_y * angle;  // ry: 绕 Y 轴旋转角度
        force_ref_frame[5] = axis_z * angle;  // rz: 绕 Z 轴旋转角度
    }

    // ---- 力控选择向量 (force_selection_vector) ----
    // {0, 0, 1, 0, 0, 0} 的含义:
    //   索引 0=X平移, 1=Y平移, 2=Z平移, 3=RX旋转, 4=RY旋转, 5=RZ旋转
    //   =1 的轴做力控，=0 的轴做位置控制
    //   此处：仅在 Z 方向做力控，其余 5 个自由度保持位置控制
    //   这意味着：Z 方向顺应表面凹凸（恒力），XY 方向严格跟随轨迹
    const ELITE::vector6int32_t force_selection_vector = {0, 0, 1, 0, 0, 0};

    // ---- 力控目标力/力矩 (force_wrench) ----
    // {0, 0, 8.0, 0, 0, 0}:
    //   Fz = 8.0N（Z 方向施加 8 牛顿的恒定接触力）
    //   其他方向力/力矩均为 0
    const ELITE::vector6d_t force_wrench = {0.0, 0.0, 8.0, 0.0, 0.0, 0.0};

    // ---- 力控位移限制 (force_limits) ----
    // {0, 0, 0.02, 0, 0, 0}:
    //   Z 方向最大顺应位移 = 0.02m = 2cm
    //   防止力控过度偏离轨迹（如深度相机误读导致工具压入工件太深）
    const ELITE::vector6d_t force_limits = {0, 0, 0.02, 0, 0, 0};

    // ===================================================================
    // 4D. 开启力控 + 执行笛卡尔轨迹
    // ===================================================================
    std::cout << "开始通过 MoveIt 执行笛卡尔轨迹..." << std::endl;
    std::cin.get();  // 操作员最后确认

    // 启动力控模式
    if (!my_controller->startForceMode(
            force_ref_frame, force_selection_vector, force_wrench, force_limits)) {
        ROS_ERROR("Failed to start Force Mode! Aborting.");
        return -1;
    }
    ROS_INFO("Force Mode started with target force Z: %.2f N", force_wrench[2]);

    // 等待力控稳定
    ros::Duration(0.5).sleep();

    // [备选方案] 直接用 SDK servoj 执行（已注释，当前选择 MoveIt 执行）
    // if (!my_controller->RunServojTrajectory(sdk_traj, traj_times)) { ... }

    // MoveIt 执行笛卡尔轨迹（力控已在后台运行）
    const auto cartesian_exec_result = move_group.execute(cartesian_plan);

    // 轨迹执行完毕后关闭力控
    const bool force_end_ok = my_controller->endForceMode();

    // 检查执行结果
    if (cartesian_exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("MoveIt execution of Cartesian trajectory failed!");
        return -1;
    }
    if (!force_end_ok) {
        ROS_ERROR("Failed to end Force Mode!");
        return -1;
    }

    ROS_INFO("Force Mode ended successfully.");
    ROS_INFO("MoveIt Cartesian trajectory finished.");

    // 等待 5 秒，让机械臂稳定、操作员观察结果
    ros::Duration(5).sleep();

    // =======================================================================
    // Step 5: 返回安全位置（收納姿态）
    // =======================================================================
    ROS_INFO("Step 5: Return to observation pose");
    std::cout << "按Enter回原点..." << std::endl;
    std::cin.get();
    move_group.setStartStateToCurrentState();
    move_group.setJointValueTarget(shouna_joints);
    move_group.move();

    // 等待 ROS 关闭信号（Ctrl+C），保持节点存活
    ros::waitForShutdown();
    return 0;
}




