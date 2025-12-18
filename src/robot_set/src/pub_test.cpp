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

const std::string DEFAULT_ROBOT_IP = "192.168.1.200";
const std::string DEFAULT_PC_IP = "192.168.1.150";
const std::string external_control_file_address = "/home/barry/workspace/climbrobot_mjk/src/robot_sdk_wrapper/resource/external_control.script";
const std::string output_recipe_file_address = "/home/barry/workspace/climbrobot_mjk/src/robot_sdk_wrapper/resource/output_recipe.txt";
const std::string input_recipe_file_address = "/home/barry/workspace/climbrobot_mjk/src/robot_sdk_wrapper/resource/input_recipe.txt";
const std::string task_file_address = "mjktest.task";


int main(int argc, char** argv)
{

    EliteCSRobotSDK cs66robot(DEFAULT_ROBOT_IP,DEFAULT_PC_IP,external_control_file_address,
                    output_recipe_file_address,input_recipe_file_address,task_file_address,250);

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

    ELITE::vector6d_t startline = {0.3 ,0 ,0.3, 3.14, 0, 3.14};
    cs66robot.moveLine(startline,5);
    std::cout << "Robot move to start successful" << std::endl;

    cs66robot.disconnect();

    return 0;
}
