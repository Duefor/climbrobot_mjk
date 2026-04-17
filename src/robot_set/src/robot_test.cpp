#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <sstream>
#include <robot_set/TCPState.h>
#include <sensor_msgs/JointState.h>


#include <robot_sdk_wrapper/robot_sdk.h>

#include <memory>
#include <thread>
#include <iostream>

const std::string DEFAULT_ROBOT_IP = "192.168.1.199";
const std::string DEFAULT_PC_IP = "192.168.1.150";
const std::string external_control_file_address = "/home/duefor/climbrobot_mjk/src/robot_sdk_wrapper/resource/external_control.script";
const std::string output_recipe_file_address = "/home/duefor/climbrobot_mjk/src/robot_sdk_wrapper/resource/output_recipe.txt";
const std::string input_recipe_file_address = "/home/duefor/climbrobot_mjk/src/robot_sdk_wrapper/resource/input_recipe.txt";
const std::string task_file_address = "mjktest.task";


int main(int argc, char** argv)
{
    ros::init(argc,argv,"test");
    ros::NodeHandle nh;
    EliteCSRobotSDK cs66robot(DEFAULT_ROBOT_IP,DEFAULT_PC_IP,true, external_control_file_address,
                    output_recipe_file_address,input_recipe_file_address,task_file_address,250);

    // if(!cs66robot.init_read_data()){
    //     std::cout << "Robot init false" << std::endl;
    //     return 1;
    // }
    // std::cout << "Robot init successful" << std::endl;

    // ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/cs66/joint_states",10);
    // ros::Rate rate = ros::Rate(200);
    // while(ros::ok())
    // {
    //     auto joint = cs66robot.getCurrentJoint();
    //     sensor_msgs::JointState msg;
    //     msg.position = {joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]};
    //     pub.publish(msg);
    //     rate.sleep();
    // }

    // EliteCSRobotSDK cs66robot(DEFAULT_ROBOT_IP,DEFAULT_PC_IP,true, external_control_file_address,
    //                 output_recipe_file_address,input_recipe_file_address,task_file_address,250);

    if(!cs66robot.init()){
        std::cout << "Robot init false" << std::endl;
        return 1;
    }
    std::cout << "Robot init successful" << std::endl;
    if(!cs66robot.start()){
        std::cout << "Robot start false" << std::endl;
        return 1;
    }
    std::cout << "Robot start successful" << std::endl;


    // ELITE::vector6d_t exforce = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    // while (true)
    // {
    //     cs66robot.test(exforce);
    //     std::this_thread::sleep_for(std::chrono::microseconds(1000));
    // }
    

    // // 读取当前关节角
    // auto current = cs66robot.getCurrentJoint();

    // if (current.size() != 6)
    // {
    //     std::cout << "Joint read failed\n";
    //     return 0;
    // }

    // // 构造轨迹
    // std::vector<EliteCSRobotSDK::TrajectoryPoint> traj;

    // EliteCSRobotSDK::TrajectoryPoint p0;
    // p0.positions = current;
    // ELITE::vector6d_t zero;
    // p0.velocities = zero;
    // p0.accelerations = zero;
    // p0.time_from_start = 0.0;

    // EliteCSRobotSDK::TrajectoryPoint p1;
    // p1.positions = current;

    // // 目标：第一个关节 +20度
    // p1.positions[0] -= 20.0 * M_PI / 180.0;

    // p1.velocities = zero;
    // p1.accelerations = zero;
    // p1.time_from_start = 2.0;   // 4秒到达

    // EliteCSRobotSDK::TrajectoryPoint p2;
    // p2.positions = p1.positions;

    // // 目标：第一个关节 +20度
    // p2.positions[0] += 20.0 * M_PI / 180.0;

    // p2.velocities = zero;
    // p2.accelerations = zero;
    // p2.time_from_start = 4.0;   // 4秒到达

    // traj.push_back(p0);
    // traj.push_back(p1);
    // traj.push_back(p2);

    // // auto start = std::chrono::steady_clock::now();
    // // cs66robot.moveJoint_servo(p1.positions,p1.time_from_start);
    // // auto end = std::chrono::steady_clock::now();
    // // std::chrono::duration<double> elapsed_seconds = end - start;
    // // std::cout << "阻塞时间: " << elapsed_seconds.count() << " 秒" << std::endl;

    // std::cout << "Start trajectory execution...\n";

    // bool ok = cs66robot.ExecuteJointTrajectory(traj, 200.0);

    // if (ok)
    //     std::cout << "Trajectory finished successfully\n";
    // else
    //     std::cout << "Trajectory failed\n";
    
    // std::this_thread::sleep_for(std::chrono::seconds(10));
    
    // auto start = std::chrono::steady_clock::now();
    // ELITE::vector6d_t startline = cs66robot.getCurrentTCPPose();
    // startline[2] += 0.05;
    // cs66robot.moveLine(startline,5.0);
    // auto end = std::chrono::steady_clock::now();
    // std::chrono::duration<double> elapsed_seconds = end - start;
    // std::cout << "阻塞时间: " << elapsed_seconds.count() << " 秒" << std::endl;

    // ELITE::vector6d_t vec{0.0,0.0,0.01,0.0,0.0,0.0};
    // cs66robot.lineSpeed(vec);
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    // auto actualvec = cs66robot.getCurrentTCPVelocity();
    // std::cout << "当前tcp速度为：";
    // for(int i=0; i<6; i++)
    // {
    //     std::cout << actualvec[i] << ",";
    // }
    // std::cout << std::endl;

    // std::this_thread::sleep_for(std::chrono::seconds(2));


    cs66robot.disconnect();

    return 0;
}
