/**
 * @file simple_servoj_example.cpp
 * @brief 简化的servoj控制示例，直接参考SDK示例
 * 
 * 本示例展示了如何使用简化的CS66RobotController进行：
 * 1. 简单的关节空间控制
 * 2. 简单的笛卡尔空间控制
 * 3. 连续发送servoj命令实现运动
 */

#include "cs66_ctl_manipulation/cs66_robot_controller.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <chrono>
#include <thread>
#include <cmath>

class SimpleServojDemo {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::unique_ptr<CS66RobotController> controller_;
    
    // 发布器
    ros::Publisher joint_pub_;
    ros::Publisher tcp_pub_;

public:
    SimpleServojDemo() : private_nh_("~") {
        // 创建控制器
        controller_ = std::make_unique<CS66RobotController>(nh_, private_nh_);
        
        // 创建发布器
        joint_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/cs66/joint_command", 10);
        tcp_pub_ = nh_.advertise<geometry_msgs::Pose>("/cs66/tcp_command", 10);
        
        ROS_INFO("Simple Servoj Demo initialized");
    }
    
    ~SimpleServojDemo() = default;
    
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
        
        // 演示1：关节空间运动（参考servoj_example.cpp）
        demonstrateJointMotion();
        
        // 演示2：笛卡尔空间运动（参考servoj_cartesian_example.cpp）
        demonstrateCartesianMotion();
        
        // 演示3：连续运动控制
        demonstrateContinuousMotion();
        
        ROS_INFO("All demonstrations completed");
    }
    
private:
    void demonstrateJointMotion() {
        ROS_INFO("=== 演示1：关节空间运动（参考servoj_example.cpp） ===");
        
        // 获取当前关节位置
        auto current_joints = controller_->getCurrentJointPositions();
        ROS_INFO("Current joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                 current_joints[0], current_joints[1], current_joints[2],
                 current_joints[3], current_joints[4], current_joints[5]);
        
        // 参考SDK示例：让第6关节正反转
        std_msgs::Float64MultiArray joint_msg;
        joint_msg.data.resize(6);
        
        // 复制当前关节位置
        for (int i = 0; i < 6; i++) {
            joint_msg.data[i] = current_joints[i];
        }
        
        ROS_INFO("Moving joint 6 in positive direction...");
        
        // 正转：增加第6关节角度
        for (int step = 0; step < 100; ++step) {
            joint_msg.data[5] = current_joints[5] + 0.01 * step; // 每次增加0.01弧度
            
            joint_pub_.publish(joint_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 50ms间隔
            
            if (!ros::ok()) break;
        }
        
        ROS_INFO("Moving joint 6 in negative direction...");
        
        // 反转：减少第6关节角度
        for (int step = 0; step < 100; ++step) {
            joint_msg.data[5] = current_joints[5] + 1.0 - 0.01 * step; // 从+1.0回到原位置
            
            joint_pub_.publish(joint_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            if (!ros::ok()) break;
        }
        
        ROS_INFO("Joint motion completed");
    }
    
    void demonstrateCartesianMotion() {
        ROS_INFO("=== 演示2：笛卡尔空间运动（参考servoj_cartesian_example.cpp） ===");
        
        // 获取当前TCP位姿
        auto current_pose = controller_->getCurrentTCPPose();
        ROS_INFO("Current TCP pose: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                 current_pose[0], current_pose[1], current_pose[2],
                 current_pose[3], current_pose[4], current_pose[5]);
        
        // 参考SDK示例：让TCP在Z方向上下移动
        geometry_msgs::Pose tcp_msg;
        
        // 设置位置
        tcp_msg.position.x = current_pose[0];
        tcp_msg.position.y = current_pose[1];
        tcp_msg.position.z = current_pose[2];
        
        // 设置姿态
        tcp_msg.orientation.x = current_pose[3];
        tcp_msg.orientation.y = current_pose[4];
        tcp_msg.orientation.z = current_pose[5];
        tcp_msg.orientation.w = 1.0;
        
        ROS_INFO("Moving TCP down...");
        
        // 向下移动
        for (int step = 0; step < 50; ++step) {
            tcp_msg.position.z = current_pose[2] - 0.002 * step; // 每次下降2mm
            
            tcp_pub_.publish(tcp_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            if (!ros::ok()) break;
        }
        
        ROS_INFO("Moving TCP up...");
        
        // 向上移动
        for (int step = 0; step < 50; ++step) {
            tcp_msg.position.z = current_pose[2] - 0.1 + 0.002 * step; // 从-0.1回到原位置
            
            tcp_pub_.publish(tcp_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            if (!ros::ok()) break;
        }
        
        ROS_INFO("TCP motion completed");
    }
    
    void demonstrateContinuousMotion() {
        ROS_INFO("=== 演示3：连续运动控制 ===");
        
        // 获取当前位置
        auto current_joints = controller_->getCurrentJointPositions();
        
        // 创建圆形轨迹
        const double radius = 0.05; // 5cm半径
        const int num_points = 50;
        
        ROS_INFO("Executing circular trajectory...");
        
        for (int i = 0; i < num_points; ++i) {
            double angle = 2.0 * M_PI * i / num_points;
            
            // 计算圆形轨迹上的点
            std_msgs::Float64MultiArray joint_msg;
            joint_msg.data.resize(6);
            
            // 保持其他关节不变，只移动第1和第2关节
            joint_msg.data[0] = current_joints[0] + radius * cos(angle);
            joint_msg.data[1] = current_joints[1] + radius * sin(angle);
            joint_msg.data[2] = current_joints[2];
            joint_msg.data[3] = current_joints[3];
            joint_msg.data[4] = current_joints[4];
            joint_msg.data[5] = current_joints[5] + angle * 0.1; // 第6关节缓慢旋转
            
            joint_pub_.publish(joint_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100ms间隔
            
            if (!ros::ok()) break;
        }
        
        ROS_INFO("Continuous motion completed");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_servoj_example");
    
    try {
        SimpleServojDemo demo;
        demo.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Demo failed: %s", e.what());
        return 1;
    }
    
    return 0;
}






