#include "../common/hik_camera.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

using namespace std;

string strSavePath;
const void getDepthImg(const sensor_msgs::ImageConstPtr& msg);
const void getRgbImg(const sensor_msgs::ImageConstPtr& msg);
const void getLeftIrImg(const sensor_msgs::ImageConstPtr& msg);
const void getRightIrImg(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hik_camera_image_subscriber"); 
    ros::NodeHandle nh;
    // 获取存图路径
    mkdir("image", S_IRWXU);
    strSavePath = "image/";
    ROS_INFO("[Subscriber] save path: %s", strSavePath.c_str());
    ros::Subscriber depthSub = nh.subscribe<sensor_msgs::Image>("/camera/depth", 1, &getDepthImg); 
    ros::Subscriber rgbSub = nh.subscribe<sensor_msgs::Image>("/camera/rgb", 1, &getRgbImg); 
    ros::Subscriber leftIrSub = nh.subscribe<sensor_msgs::Image>("/camera/leftIr", 1, &getLeftIrImg); 
    ros::Subscriber rightIrSub = nh.subscribe<sensor_msgs::Image>("/camera/rightIr", 1, &getRightIrImg); 

    ros::spin();
    return 0;
}

const void getDepthImg(const sensor_msgs::ImageConstPtr& msg)
{
    // ROS_INFO("[Subscriber] Get depht image [%d] success", msg->header.seq);
    cv_bridge::CvImagePtr depthPtr;
    try
    {
        depthPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("[Subscriber] cv_bridge depth image Error! %s", e.what());
        depthPtr = nullptr;
    }
    if(nullptr != depthPtr)
    {
        // 保存深度图
        char chFileName[256] = "";
        // sprintf(chFileName, "%s/[%d]_DepthImage.png", strSavePath.c_str(), msg->header.seq);s
        cv::imwrite(chFileName, depthPtr->image);
        // ROS_INFO("[Subscriber] Save depht image [%d] success", msg->header.seq);
    }
}

const void getRgbImg(const sensor_msgs::ImageConstPtr& msg)
{
    // ROS_INFO("[Subscriber] Get rgb image [%d] success", msg->header.seq);
    cv_bridge::CvImagePtr rgbPtr;
    try
    {
        rgbPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("[Subscriber] cv_bridge rgb image Error! %s", e.what());
        rgbPtr = nullptr;
    }
    if(nullptr != rgbPtr)
    {
        // 保存彩色图
        char chFileName[256] = "";
        // sprintf(chFileName, "%s/[%d]_RgbImage.png", strSavePath.c_str(), msg->header.seq);
        cv::imwrite(chFileName, rgbPtr->image);
        // ROS_INFO("[Subscriber] Save rgb image [%d] success", msg->header.seq);
    }
}

const void getLeftIrImg(const sensor_msgs::ImageConstPtr& msg)
{
    // ROS_INFO("[Subscriber] Get left ir image [%d] success", msg->header.seq);
    cv_bridge::CvImagePtr leftIrPtr;
    try
    {
        leftIrPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("[Subscriber] cv_bridge left ir image Error! %s", e.what());
        leftIrPtr = nullptr;
    }
    if(nullptr != leftIrPtr)
    {
        // 保存左目IR图
        char chFileName[256] = "";
        // sprintf(chFileName, "%s/[%d]_LeftIrImage.png", strSavePath.c_str(), msg->header.seq);
        cv::imwrite(chFileName, leftIrPtr->image);
        // ROS_INFO("[Subscriber] Save left ir image [%d] success", msg->header.seq);
    }
}

const void getRightIrImg(const sensor_msgs::ImageConstPtr& msg)
{
    // ROS_INFO("[Subscriber] Get right ir image [%d] success", msg->header.seq);
    cv_bridge::CvImagePtr rightIrPtr;
    try
    {
        rightIrPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch(const std::exception& e)
    {
        // ROS_ERROR("[Subscriber] cv_bridge right ir image Error! %s", e.what());
        rightIrPtr = nullptr;
    }
    if(nullptr != rightIrPtr)
    {
        // 保存右目IR图
        char chFileName[256] = "";
        // sprintf(chFileName, "%s/[%d]_rightIrImage.png", strSavePath.c_str(), msg->header.seq);
        cv::imwrite(chFileName, rightIrPtr->image);
        // ROS_INFO("[Subscriber] Save right ir image [%d] success", msg->header.seq);
    }
}