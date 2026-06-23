#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "../common/hik_camera.h"
#include "mv3d_rgbd_ros/image.h"
#include "mv3d_rgbd_ros/imageRequest.h"
#include "mv3d_rgbd_ros/imageResponse.h"

using namespace std;

string strSavePath;
bool SaveImage(mv3d_rgbd_ros::image::Request &request, mv3d_rgbd_ros::image::Response &response);

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 初始化ros节点
    ros::init(argc, argv, "hik_camera_image_server");
    ros::NodeHandle nh;
    // 获取存图路径
    mkdir("image", S_IRWXU);
    strSavePath = "image/";
    ROS_INFO("[Server] save path: %s", strSavePath.c_str());
    // 建立服务器
    ros::ServiceServer server;
    ROS_INFO("---------------- Server start ----------------");
    server = nh.advertiseService("SaveImage", SaveImage);
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}

// 设置回调函数
bool SaveImage(mv3d_rgbd_ros::image::Request &request, mv3d_rgbd_ros::image::Response &response)
{
    // 获取帧号
    uint32_t nFrameNum = request.frameNum;
    // 解析图像类型
    uint32_t nMsgImgType = request.typeInfo;
    // 获取相机序列号
    string strSerialNum = request.camSerialNum;
    if(nMsgImgType & TYPE_DEPTH)
    {
        ROS_INFO("[Server] SN:%s Get depth image[%d] success", strSerialNum.c_str(), nFrameNum);
        cv_bridge::CvImagePtr depthPtr;
        try
        {
            depthPtr = cv_bridge::toCvCopy(request.depthImg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("[Server] SN:%s cv_bridge depth image Error! %s", strSerialNum.c_str(), e.what());
            depthPtr = nullptr;
        }
        if(nullptr != depthPtr)
        {
            // 保存深度图
            char chFileName[256] = "";
            sprintf(chFileName, "%s/[%d]_DepthImage.png", strSavePath.c_str(), nFrameNum);
            cv::imwrite(chFileName, depthPtr->image);
            ROS_INFO("[Server] SN:%s Save [%d]_DepthImage.png success", strSerialNum.c_str(), nFrameNum);
        }
    }
    if(nMsgImgType & TYPE_RGB)
    {
        ROS_INFO("[Server] SN:%s Get rgb image[%d] success", strSerialNum.c_str(), nFrameNum);
        cv_bridge::CvImagePtr rgbPtr;
        try
        {
            rgbPtr = cv_bridge::toCvCopy(request.rgbImg, sensor_msgs::image_encodings::BGR8);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("[Server] SN:%s cv_bridge rgb image Error! %s", strSerialNum.c_str(), e.what());
            rgbPtr = nullptr;
        }
        if(nullptr != rgbPtr)
        {
            // 保存彩色图
            char chFileName[256] = "";
            sprintf(chFileName, "%s/[%d]_RgbImage.png", strSavePath.c_str(), nFrameNum);
            cv::imwrite(chFileName, rgbPtr->image);
            ROS_INFO("[Server] SN:%s Save [%d]_RgbImage.png success", strSerialNum.c_str(), nFrameNum);
        }
    }
    if(nMsgImgType & TYPE_LEFT_IR)
    {
        ROS_INFO("[Server] SN:%s Get left ir image[%d] success", strSerialNum.c_str(), nFrameNum);
        cv_bridge::CvImagePtr leftIrPtr;
        try
        {
            leftIrPtr = cv_bridge::toCvCopy(request.leftIrImg, sensor_msgs::image_encodings::TYPE_8UC1);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("[Server] SN:%scv_bridge leftIr image Error! %s", strSerialNum.c_str(), e.what());
            leftIrPtr = nullptr;
        }
        if(nullptr != leftIrPtr)
        {
            // 保存左目IR图
            char chFileName[256] = "";
            sprintf(chFileName, "%s/[%d]_LeftIrImage.png", strSavePath.c_str(), nFrameNum);
            cv::imwrite(chFileName, leftIrPtr->image);
            ROS_INFO("[Server] SN:%s Save [%d]_LeftIrImage.png success", strSerialNum.c_str(), nFrameNum);
        }
    }
    if(nMsgImgType & TYPE_RIGHT_IR)
    {
        ROS_INFO("[Server] SN:%s Get right ir image[%d] success", strSerialNum.c_str(), nFrameNum);
        cv_bridge::CvImagePtr rightIrPtr;
        try
        {
            rightIrPtr = cv_bridge::toCvCopy(request.rightIrImg, sensor_msgs::image_encodings::TYPE_8UC1);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("[Server] SN:%s cv_bridge rightIr image Error! %s", strSerialNum.c_str(), e.what());
            rightIrPtr = nullptr;
        }
        if(nullptr != rightIrPtr)
        {
            // 保存右目IR图
            char chFileName[256] = "";
            sprintf(chFileName, "%s/[%d]_RightIrImage.png", strSavePath.c_str(), nFrameNum);
            cv::imwrite(chFileName, rightIrPtr->image);
            ROS_INFO("[Server] SN:%s Save [%d]_RightIrImage.png success", strSerialNum.c_str(), nFrameNum);
        }
    }
    return true;
}
