/*
订阅一个持续发布的关节角度话题
每次接收到关节角度数据，就作为机械臂新的目标
持续调用接口驱动机械臂到该目标
*/

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <sstream>
#include <sensor_msgs/JointState.h>


#include <robot_sdk_wrapper/robot_sdk.h>

#include <memory>
#include <thread>
#include <iostream>
#include <mutex>


const std::string DEFAULT_ROBOT_IP = "192.168.1.200";
const std::string DEFAULT_PC_IP = "192.168.1.150";
const std::string external_control_file_address = "external_control.script";
const std::string output_recipe_file_address = "output_recipe.txt";
const std::string input_recipe_file_address = "input_recipe.txt";
const std::string task_file_address = "mjktest.task";


bool is_move_finish = false;

ELITE::vector6d_t latest_target = {0,0,0,0,0,0};   //设置为默认位姿的关节角，避免意外
std::mutex target_mutex;    // 互斥锁保护
bool has_target = false;

// 工具函数：vector 转字符串
std::string ToString(const ELITE::vector6d_t& vec)
{
    std::ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < vec.size(); ++i)
    {
        ss << vec[i];
        if (i != vec.size() - 1) ss << ", ";
    }
    ss << "]";
    return ss.str();
}

// 函数：驱动机械臂运动
void moveArmToJointPositions(EliteCSRobotSDK* robot)
{
    while(ros::ok())
    {
        if(has_target)
        {
            ELITE::vector6d_t latest_target_copy;
            {
                std::lock_guard<std::mutex> lock(target_mutex);
                latest_target_copy = latest_target; // 拷贝一份出来用
            }
            robot->moveJoint(latest_target_copy,1,0);
        }
    }

}

// 订阅回调：收到新关节角度时立即更新目标点
void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (msg->position.size() != 6)
    {
        ROS_WARN("Received joint data size = %lu, expected 6.", msg->position.size());
        return;
    }
    // 更新最新关节角
    std::lock_guard<std::mutex> lock(target_mutex);
    for (size_t i = 0; i < 6; ++i)
    {
        latest_target[i] = msg->position[i];
    }
    has_target = true; // 标记收到过目标
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_teleop_follower_node");
    ros::NodeHandle nh;

    EliteCSRobotSDK cs66robot(DEFAULT_ROBOT_IP,DEFAULT_PC_IP,external_control_file_address,output_recipe_file_address,input_recipe_file_address,task_file_address,250);

    if(!cs66robot.init()){
        std::cout << "Robot init false" << std::endl;
        return 1;
    }
    if(!cs66robot.start()){
        std::cout << "Robot start false" << std::endl;
        return 1;
    }

    // 订阅目标关节角度话题
    ros::Subscriber sub = nh.subscribe("/joint_states_target", 10, jointCallback);

    std::thread control_thread(moveArmToJointPositions, &cs66robot);

    ROS_INFO("Joint teleoperation follower started, listening to /joint_states_target");

    ros::spin();
    // 等待控制线程结束
    control_thread.join();
    // 断开远程连接
    cs66robot.disconnect();

    return 0;
}
