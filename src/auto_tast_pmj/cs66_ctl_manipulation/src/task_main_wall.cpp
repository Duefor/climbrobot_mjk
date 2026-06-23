// 该代码作为ocr的客户端，请求检测服务得到视觉轨迹点，然后将轨迹点做
#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
// 引入刚才改好的两个类
#include "cs66_robot_controller.h"
#include "cs66_follow_joint_traj_server.h" 

#include "mv3d_rgbd_ros/DetectSteelStamp.h" 

#include <cmath> 

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

// 辅助函数：Catmull-Rom 插值 (一维)
double CatmullRom(double p0, double p1, double p2, double p3, double t) {
    double v0 = (p2 - p0) * 0.5;
    double v1 = (p3 - p1) * 0.5;
    double t2 = t * t;
    double t3 = t * t * t;
    return (2 * p1 - 2 * p2 + v0 + v1) * t3 + (-3 * p1 + 3 * p2 - 2 * v0 - v1) * t2 + v0 * t + p1;
}

// /**
//  * @brief 空间轨迹平滑加密函数
//  * @param sparse_poses OCR识别出的稀疏路点 (间隔18cm)
//  * @param step_size 插值步长，默认 0.005 (5mm)
//  * @return std::vector<geometry_msgs::Pose> 加密后的密集路点
//  */
// std::vector<geometry_msgs::Pose> GenerateSmoothPath(
//     const std::vector<geometry_msgs::Pose>& sparse_poses, 
//     double step_size = 0.005) 
// {
//     std::vector<geometry_msgs::Pose> dense_poses;
//     if (sparse_poses.size() < 2) return sparse_poses;

//     std::vector<geometry_msgs::Pose> P = sparse_poses;

//     // --- 优化：智能计算首尾虚拟控制点 ---
//     // 目的：让曲线在起点和终点顺着趋势延伸，而不是强制变直
    
//     // 1. 头部虚拟点：P_virtual_start = P0 - (P1 - P0)
//     // 也就是向 P0 -> P1 的反方向延伸一点
//     geometry_msgs::Pose p_head = sparse_poses.front();
//     p_head.position.x -= (sparse_poses[1].position.x - sparse_poses[0].position.x);
//     p_head.position.y -= (sparse_poses[1].position.y - sparse_poses[0].position.y);
//     p_head.position.z -= (sparse_poses[1].position.z - sparse_poses[0].position.z);
//     P.insert(P.begin(), p_head);

//     // 2. 尾部虚拟点：P_virtual_end = Pn + (Pn - Pn-1)
//     size_t n = sparse_poses.size();
//     geometry_msgs::Pose p_tail = sparse_poses.back();
//     p_tail.position.x += (sparse_poses[n-1].position.x - sparse_poses[n-2].position.x);
//     p_tail.position.y += (sparse_poses[n-1].position.y - sparse_poses[n-2].position.y);
//     p_tail.position.z += (sparse_poses[n-1].position.z - sparse_poses[n-2].position.z);
//     P.push_back(p_tail);

//     // 2. 遍历每一段 (从 P[1] 到 P[N-1])
//     // 原始点 i 对应 P[i+1]
//     for (size_t i = 0; i < sparse_poses.size() - 1; ++i) {
        
//         // 准备4个控制点 (位置)
//         const auto& p0 = P[i].position;     // Prev
//         const auto& p1 = P[i+1].position;   // Start
//         const auto& p2 = P[i+2].position;   // End
//         const auto& p3 = P[i+3].position;   // Next
//         // 准备2个控制点 (姿态 - 用于 Slerp)
//         tf2::Quaternion q1, q2;
//         tf2::fromMsg(P[i+1].orientation, q1);
//         tf2::fromMsg(P[i+2].orientation, q2);

//         // 计算这一段的欧氏距离
//         double dist = std::sqrt(std::pow(p2.x - p1.x, 2) + 
//                                 std::pow(p2.y - p1.y, 2) + 
//                                 std::pow(p2.z - p1.z, 2));

//         int steps = std::max(1, (int)(dist / step_size));

//         for (int s = 0; s < steps; ++s) {
//             double t = (double)s / steps; // 0.0 ~ 0.99...

//             geometry_msgs::Pose new_pose;

//             new_pose.position.x = CatmullRom(p0.x, p1.x, p2.x, p3.x, t);
//             new_pose.position.y = CatmullRom(p0.y, p1.y, p2.y, p3.y, t);
//             new_pose.position.z = CatmullRom(p0.z, p1.z, p2.z, p3.z, t);

//             tf2::Quaternion q_interp = q1.slerp(q2, t);
//             new_pose.orientation = tf2::toMsg(q_interp);

//             dense_poses.push_back(new_pose);
//         }
//     }
//     dense_poses.push_back(sparse_poses.back());

//     return dense_poses;
// }
std::vector<geometry_msgs::Pose> GenerateSmoothPath(
    const std::vector<geometry_msgs::Pose>& sparse_poses, 
    double step_size = 0.005) 
{
    std::vector<geometry_msgs::Pose> dense_poses;
    if (sparse_poses.size() < 2) return sparse_poses;

    std::vector<geometry_msgs::Pose> P = sparse_poses;
    size_t n = sparse_poses.size();

    // ==========================================
    // 优化：智能计算首尾虚拟控制点 (保持曲率，而非直线)
    // ==========================================

    // --- 1. 计算头部虚拟点 (P_head) ---
    geometry_msgs::Pose p_head = sparse_poses.front();
    if (n >= 3) {
        // 使用前三个点计算变化趋势：v1 = P1-P0, v2 = P2-P1
        // 假设趋势反向延续：P_head = P0 - (v1 + (v1 - v2)) = P0 - 2*v1 + v2
        // 简单说就是：如果 P0-P1-P2 是弯的，P_head 也要弯过去
        p_head.position.x = sparse_poses[0].position.x - 2*(sparse_poses[1].position.x - sparse_poses[0].position.x) + (sparse_poses[2].position.x - sparse_poses[1].position.x);
        p_head.position.y = sparse_poses[0].position.y - 2*(sparse_poses[1].position.y - sparse_poses[0].position.y) + (sparse_poses[2].position.y - sparse_poses[1].position.y);
        p_head.position.z = sparse_poses[0].position.z - 2*(sparse_poses[1].position.z - sparse_poses[0].position.z) + (sparse_poses[2].position.z - sparse_poses[1].position.z);
    } else {
        // 点不够时退化为线性
        p_head.position.x -= (sparse_poses[1].position.x - sparse_poses[0].position.x);
        p_head.position.y -= (sparse_poses[1].position.y - sparse_poses[0].position.y);
        p_head.position.z -= (sparse_poses[1].position.z - sparse_poses[0].position.z);
    }
    P.insert(P.begin(), p_head);

    // --- 2. 计算尾部虚拟点 (P_tail) ---
    // 解决 "4到5走直线跑偏" 的核心代码
    geometry_msgs::Pose p_tail = sparse_poses.back();
    if (n >= 3) {
        // P_last = sparse_poses[n-1]
        // P_prev = sparse_poses[n-2]
        // P_prev2 = sparse_poses[n-3]
        
        // v_last = P_last - P_prev
        // v_prev = P_prev - P_prev2
        // diff = v_last - v_prev (加速度/曲率变化量)
        // P_tail = P_last + v_last + diff = P_last + 2*v_last - v_prev
        
        double x_last = sparse_poses[n-1].position.x; double y_last = sparse_poses[n-1].position.y; double z_last = sparse_poses[n-1].position.z;
        double x_prev = sparse_poses[n-2].position.x; double y_prev = sparse_poses[n-2].position.y; double z_prev = sparse_poses[n-2].position.z;
        double x_prev2 = sparse_poses[n-3].position.x; double y_prev2 = sparse_poses[n-3].position.y; double z_prev2 = sparse_poses[n-3].position.z;

        p_tail.position.x = x_last + 2*(x_last - x_prev) - (x_prev - x_prev2);
        p_tail.position.y = y_last + 2*(y_last - y_prev) - (y_prev - y_prev2);
        p_tail.position.z = z_last + 2*(z_last - z_prev) - (z_prev - z_prev2);
    } else {
        // 点不够时退化为线性
        p_tail.position.x += (sparse_poses[n-1].position.x - sparse_poses[n-2].position.x);
        p_tail.position.y += (sparse_poses[n-1].position.y - sparse_poses[n-2].position.y);
        p_tail.position.z += (sparse_poses[n-1].position.z - sparse_poses[n-2].position.z);
    }
    // 姿态直接沿用最后一个点的姿态，不做旋转插值预测，防止翻转
    p_tail.orientation = sparse_poses.back().orientation; 
    P.push_back(p_tail);

    // -----------------------------------------------------------
    // 下面的 for 循环逻辑保持不变 ...
    // -----------------------------------------------------------
    for (size_t i = 0; i < sparse_poses.size() - 1; ++i) {
        const auto& p0 = P[i].position;     
        const auto& p1 = P[i+1].position;   
        const auto& p2 = P[i+2].position;   
        const auto& p3 = P[i+3].position;   

        tf2::Quaternion q1, q2;
        tf2::fromMsg(P[i+1].orientation, q1);
        tf2::fromMsg(P[i+2].orientation, q2);

        double dist = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2) + std::pow(p2.z - p1.z, 2));
        int steps = std::max(1, (int)(dist / step_size));

        for (int s = 0; s < steps; ++s) {
            double t = (double)s / steps; 

            geometry_msgs::Pose new_pose;
            new_pose.position.x = CatmullRom(p0.x, p1.x, p2.x, p3.x, t);
            new_pose.position.y = CatmullRom(p0.y, p1.y, p2.y, p3.y, t);
            new_pose.position.z = CatmullRom(p0.z, p1.z, p2.z, p3.z, t);

            tf2::Quaternion q_interp = q1.slerp(q2, t);
            new_pose.orientation = tf2::toMsg(q_interp);

            dense_poses.push_back(new_pose);
        }
    }
    dense_poses.push_back(sparse_poses.back());

    return dense_poses;
}

std::vector<ELITE::vector6d_t> ConvertTrajectoryToSDK(
    const moveit_msgs::RobotTrajectory& plan_traj, 
    std::vector<double>& times) 
{
    std::vector<ELITE::vector6d_t> sdk_traj;
    const auto& points = plan_traj.joint_trajectory.points;
    
    for (const auto& p : points) {
        ELITE::vector6d_t q; 
        for (size_t i = 0; i < 6 && i < p.positions.size(); ++i) {
            q[i] = p.positions[i];
        }
        sdk_traj.push_back(q);
        times.push_back(p.time_from_start.toSec());
    }
    return sdk_traj;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "steel_task_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    // 全局唯一controller实例
    auto my_controller = std::make_shared<CS66RobotController>(nh, pnh);
    
    // 等待连接
    ros::Duration(1.0).sleep();
    CS66FollowTrajectoryServer moveit_server(nh, my_controller);

    // 启动多线程 Spinner，在后台开线程处理这些回调
    ros::AsyncSpinner spinner(2);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("arm_group");
    // move_group.setPlannerId("BITstar"); 
    // move_group.setPlanningTime(8.0);    
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.2);
    
    // --- 阶段一：回相机观测位置（待定，现在还不太确定在壁面上该角度是否合适） ---
    ROS_INFO("Step 1: Go Home");
    // std::vector<double> init_joints = {0, -1.69, -2.47, -0.42, 1.57, 0};
    std::vector<double> init_joints = {0, -1.228, -1.744, -1.640, 1.568, 0};
    // std::vector<double> init_joints = {-0.06,-1.57,-1.916,-1.01,1.68,0};


    std::cout << "按Enter继续移动至观测点位..." << std::endl;
    std::cin.get();
    move_group.setJointValueTarget(init_joints);
    move_group.move();

    // --- 阶段二：调用 OCR 获取坐标 ---
    std::cout << "按Enter启动OCR detection..." << std::endl;
    std::cin.get();

    ROS_INFO("Step 2: OCR Detection");
    ros::ServiceClient ocr_client = nh.serviceClient<mv3d_rgbd_ros::DetectSteelStamp>("get_steel_stamp_location");
    mv3d_rgbd_ros::DetectSteelStamp srv;
    if (!ocr_client.call(srv)){
        ROS_ERROR("Failed to call OCR service");
        return -1;
    }
    std::vector<geometry_msgs::Pose> target_poses = srv.response.poses;
    ROS_INFO("=== Debugging Target Poses（先去debug_images里看看识别的是否准确） ===");
    for (size_t i = 0; i < target_poses.size(); ++i) {
        const auto& p = target_poses[i];
        
        // 打印位置
        ROS_INFO("Pose[%lu] Position: [x: %.4f, y: %.4f, z: %.4f]", 
                i, p.position.x, p.position.y, p.position.z);
        
        // 打印原始四元数
        ROS_INFO("Pose[%lu] Quat: [x: %.4f, y: %.4f, z: %.4f, w: %.4f]", 
                i, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
        ROS_INFO("---------------------------------------------");
    }

    // --- 阶段三：移动到第一个点上方 (使用 MoveIt 导航) ---
    // geometry_msgs::Pose approach = target_poses[0];
    // approach.position.z += 0.1; // 上方 10cm
    // std::cout << "按Enter移动到第一个点上方..." << std::endl;
    // std::cin.get();

    // move_group.setPoseTarget(approach);
    // move_group.move();

    // --- 阶段四：插值加时间参数化，最后通过moveit执行 ---

    std::cout << "按Enter开始跟踪检测..." << std::endl;
    std::cin.get();
    std::vector<geometry_msgs::Pose> raw_waypoints;
        // 1. 先构建稀疏的关键点列表 (带 Z 轴偏移)
        for (const auto& p : target_poses) {
                geometry_msgs::Pose pose = p;
                pose.position.z += 0.1; // 保持 5cm 高度
                raw_waypoints.push_back(pose);
        }

        ROS_INFO("Generating Smooth Curve from %zu sparse points...", raw_waypoints.size());

        // 2. 调用插值函数生成模拟曲面的密集点
        std::vector<geometry_msgs::Pose> dense_waypoints = GenerateSmoothPath(raw_waypoints, 0.005);
        ROS_INFO("Generated %zu dense points for curved execution.", dense_waypoints.size());

        // 3. MoveIt 规划
        moveit_msgs::RobotTrajectory trajectory_msg;
        double fraction = move_group.computeCartesianPath(dense_waypoints, 0.005, trajectory_msg);
        if (fraction < 0.9) {
        ROS_WARN("Cartesian path only achieved %.2f%% coverage. Please check constraints.", fraction * 100.0);}

        // moveit时间参数化执行
        robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "arm_group");
        rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_msg);
        double target_velocity_scaling = 0.05;
        double target_acceleration_scaling = 0.15;
        trajectory_processing::IterativeParabolicTimeParameterization iptp;

        bool success = iptp.computeTimeStamps(rt, target_velocity_scaling, target_acceleration_scaling);

        if (success) {
            ROS_INFO("Path planned & timed successfully. Executing...");
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            rt.getRobotTrajectoryMsg(my_plan.trajectory_);
        
            move_group.execute(my_plan);
            } else {
            ROS_ERROR("Failed to compute time stamps");
            }

        // --- 阶段五：回到观测点 ---
        std::cout << "按Enter回原点..." << std::endl;
        std::cin.get();
        
        move_group.setJointValueTarget(init_joints);
        move_group.move();

        ros::waitForShutdown();
        return 0;
}