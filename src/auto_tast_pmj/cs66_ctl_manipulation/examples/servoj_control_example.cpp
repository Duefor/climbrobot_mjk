/**
 * @file servoj_control_example.cpp
 * @brief 演示基于writeServoj的高精度实时控制示例
 * 
 * 本示例展示了如何使用CS66RobotController进行：
 * 1. 实时关节空间控制
 * 2. 实时笛卡尔空间控制
 * 3. 平滑轨迹执行
 * 4. 实时线程优化
 */

#include "cs66_ctl_manipulation/cs66_robot_controller.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <chrono>
#include <thread>

class ServojControlDemo {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::unique_ptr<CS66RobotController> controller_;
    
    // 演示参数
    double motion_duration_;
    double servoj_frequency_;
    bool enable_realtime_;

public:
    ServojControlDemo() : private_nh_("~") {
        // 加载参数
        private_nh_.param<double>("motion_duration", motion_duration_, 3.0);
        private_nh_.param<double>("servoj_frequency", servoj_frequency_, 250.0);
        private_nh_.param<bool>("enable_realtime", enable_realtime_, true);
        
        // 创建控制器
        controller_ = std::make_unique<CS66RobotController>(nh_, private_nh_);
        
        ROS_INFO("Servoj Control Demo initialized");
        ROS_INFO("Motion duration: %.2f s, Servoj frequency: %.1f Hz, Realtime: %s", 
                 motion_duration_, servoj_frequency_, enable_realtime_ ? "enabled" : "disabled");
    }
    
    ~ServojControlDemo() = default;
    
    void run() {
        // 等待控制器初始化
        ros::Rate rate(10);
        while (ros::ok() && !controller_->isReady()) {
            ros::spinOnce();
            rate.sleep();
        }
        
        if (!controller_->isReady()) {
            ROS_ERROR("Failed to initialize robot controller");
            return;
        }
        
        ROS_INFO("Robot controller ready, starting demonstrations...");
        
        // 演示1：关节空间平滑运动
        demonstrateJointSpaceMotion();
        
        // 演示2：笛卡尔空间平滑运动
        demonstrateCartesianSpaceMotion();
        
        // 演示3：复杂轨迹执行
        demonstrateTrajectoryExecution();
        
        // 演示4：实时控制性能测试
        demonstrateRealtimePerformance();
        
        ROS_INFO("All demonstrations completed");
    }
    
private:
    void demonstrateJointSpaceMotion() {
        ROS_INFO("=== 演示1：关节空间平滑运动 ===");
        
        // 获取当前关节位置
        auto current_joints = controller_->getCurrentJointPositions();
        ROS_INFO("Current joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                 current_joints[0], current_joints[1], current_joints[2],
                 current_joints[3], current_joints[4], current_joints[5]);
        
        // 定义目标关节位置（小幅移动）
        std::vector<double> target_joints = {
            current_joints[0] + 0.1,  // 关节1
            current_joints[1] + 0.1,  // 关节2
            current_joints[2] - 0.1,  // 关节3
            current_joints[3] + 0.2,  // 关节4
            current_joints[4] - 0.1,  // 关节5
            current_joints[5] + 0.3   // 关节6
        };
        
        ROS_INFO("Moving to target joint positions...");
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // 执行平滑关节运动
        bool success = controller_->moveToJointPositions(target_joints, motion_duration_, true);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (success) {
            ROS_INFO("Joint motion completed successfully in %ld ms", duration.count());
        } else {
            ROS_ERROR("Joint motion failed");
        }
        
        // 等待运动完成
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    void demonstrateCartesianSpaceMotion() {
        ROS_INFO("=== 演示2：笛卡尔空间平滑运动 ===");
        
        // 获取当前TCP位姿
        auto current_pose = controller_->getCurrentTCPPose();
        ROS_INFO("Current TCP pose: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                 current_pose[0], current_pose[1], current_pose[2],
                 current_pose[3], current_pose[4], current_pose[5]);
        
        // 定义目标TCP位姿（小幅移动）
        std::vector<double> target_pose = {
            current_pose[0] + 0.05,  // x
            current_pose[1] + 0.05,  // y
            current_pose[2] + 0.05,  // z
            current_pose[3],         // rx
            current_pose[4],         // ry
            current_pose[5] + 0.1    // rz
        };
        
        ROS_INFO("Moving to target TCP pose...");
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // 执行平滑TCP运动
        bool success = controller_->moveToTCPPose(target_pose, motion_duration_, true);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (success) {
            ROS_INFO("TCP motion completed successfully in %ld ms", duration.count());
        } else {
            ROS_ERROR("TCP motion failed");
        }
        
        // 等待运动完成
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    void demonstrateTrajectoryExecution() {
        ROS_INFO("=== 演示3：复杂轨迹执行 ===");
        
        // 获取当前位姿作为起点
        auto start_pose = controller_->getCurrentTCPPose();
        
        // 定义轨迹点（圆形轨迹）
        std::vector<std::vector<double>> trajectory;
        const int num_points = 20;
        const double radius = 0.05;
        
        for (int i = 0; i <= num_points; ++i) {
            double angle = 2.0 * M_PI * i / num_points;
            std::vector<double> point = {
                start_pose[0] + radius * cos(angle),  // x
                start_pose[1] + radius * sin(angle),  // y
                start_pose[2],                        // z
                start_pose[3],                        // rx
                start_pose[4],                        // ry
                start_pose[5] + angle                 // rz
            };
            trajectory.push_back(point);
        }
        
        ROS_INFO("Executing circular trajectory with %zu points...", trajectory.size());
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // 执行轨迹
        bool success = controller_->executeTrajectory(trajectory, motion_duration_ * 2, true);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (success) {
            ROS_INFO("Trajectory execution completed successfully in %ld ms", duration.count());
        } else {
            ROS_ERROR("Trajectory execution failed");
        }
        
        // 等待运动完成
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    void demonstrateRealtimePerformance() {
        if (!enable_realtime_) {
            ROS_INFO("=== 演示4：实时控制性能测试（跳过 - 实时模式未启用） ===");
            return;
        }
        
        ROS_INFO("=== 演示4：实时控制性能测试 ===");
        
        // 测试高频控制性能
        const int test_cycles = 100;
        auto start_time = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < test_cycles; ++i) {
            // 生成小幅随机运动
            auto current_joints = controller_->getCurrentJointPositions();
            std::vector<double> target_joints = {
                current_joints[0] + (rand() % 100 - 50) * 0.001,  // ±0.05 rad
                current_joints[1] + (rand() % 100 - 50) * 0.001,
                current_joints[2] + (rand() % 100 - 50) * 0.001,
                current_joints[3] + (rand() % 100 - 50) * 0.001,
                current_joints[4] + (rand() % 100 - 50) * 0.001,
                current_joints[5] + (rand() % 100 - 50) * 0.001
            };
            
            // 快速运动（非平滑）
            controller_->moveToJointPositions(target_joints, 0.1, false);
            
            // 等待一个控制周期
            std::this_thread::sleep_for(std::chrono::microseconds(4000)); // 4ms
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        double avg_cycle_time = total_duration.count() / static_cast<double>(test_cycles);
        
        ROS_INFO("Realtime performance test completed:");
        ROS_INFO("  Total cycles: %d", test_cycles);
        ROS_INFO("  Total time: %ld ms", total_duration.count());
        ROS_INFO("  Average cycle time: %.2f ms", avg_cycle_time);
        ROS_INFO("  Target cycle time: 4.0 ms");
        ROS_INFO("  Performance: %.1f%%", 4.0 / avg_cycle_time * 100);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "servoj_control_example");
    
    try {
        ServojControlDemo demo;
        demo.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Demo failed: %s", e.what());
        return 1;
    }
    
    return 0;
}


