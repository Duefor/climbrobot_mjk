#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "mv3d_rgbd_ros/image_pointcloud.h"
#include "mv3d_rgbd_ros/image_pointcloudRequest.h"
#include "mv3d_rgbd_ros/image_pointcloudResponse.h"

#include "../common/hik_camera.h"

using namespace std;

string g_strPointCloudSavePath;
string g_strImageSavePath;

bool SaveImagePointCloud(mv3d_rgbd_ros::image_pointcloud::Request &request,
                         mv3d_rgbd_ros::image_pointcloud::Response &response);

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    // 初始化ros节点
    ros::init(argc, argv, "hik_camera_image_pointcloud_server");
    ros::NodeHandle nh;

    // 获取存点云路径（与 src 同级目录下）
    mkdir("point_cloud", S_IRWXU);
    g_strPointCloudSavePath = "point_cloud/";
    ROS_INFO("[Server] point cloud save path: %s", g_strPointCloudSavePath.c_str());

    // 获取存图路径（与 src 同级目录下）
    mkdir("image", S_IRWXU);
    g_strImageSavePath = "image/";
    ROS_INFO("[Server] image save path: %s", g_strImageSavePath.c_str());

    // 建立服务器
    ros::ServiceServer server;
    ROS_INFO("---------------- SaveImagePointCloud Server start ----------------");
    server = nh.advertiseService("SaveImagePointCloud", SaveImagePointCloud);
    
    //轮询处理挂在这个节点上的所有回调，包括 service
    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}

bool SaveImagePointCloud(mv3d_rgbd_ros::image_pointcloud::Request &request,
                         mv3d_rgbd_ros::image_pointcloud::Response &response)
{
    // 获取帧号
    uint32_t nFrameNum = request.frameNum;
    // 点云类型
    uint32_t nMsgPointCloudType = request.typeInfo;
    // 获取相机序列号
    string strSerialNum = request.camSerialNum;

    // 先保存点云
    pcl::PCDWriter pcdWriter;
    char chFileName[256] = "";

    if (nMsgPointCloudType == TYPE_POINT_CLOUD)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(request.pointCloud, *pclPointCloud);
        sprintf(chFileName, "%s/[%d]_point_cloud.pcd", g_strPointCloudSavePath.c_str(), nFrameNum);
        pcdWriter.write(chFileName, *pclPointCloud, false);
        ROS_INFO("[Server] SN:%s save [%d]_point_cloud.pcd success", strSerialNum.c_str(), nFrameNum);
    }
    else if (nMsgPointCloudType == TYPE_POINT_CLOUD_WITH_NORMALS)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr pclPointCloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::fromROSMsg(request.pointCloud, *pclPointCloudWithNormals);
        sprintf(chFileName, "%s/[%d]_point_cloud_with_normals.pcd", g_strPointCloudSavePath.c_str(), nFrameNum);
        pcdWriter.write(chFileName, *pclPointCloudWithNormals, false);
        ROS_INFO("[Server] SN:%s save [%d]_point_cloud_with_normals.pcd success", strSerialNum.c_str(), nFrameNum);
    }
    else if (nMsgPointCloudType == TYPE_TEXTURED_POINT_CLOUD)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclTexturedPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(request.pointCloud, *pclTexturedPointCloud);
        sprintf(chFileName, "%s/[%d]_textured_point_cloud.pcd", g_strPointCloudSavePath.c_str(), nFrameNum);
        pcdWriter.write(chFileName, *pclTexturedPointCloud, false);
        ROS_INFO("[Server] SN:%s save [%d]_textured_point_cloud.pcd success", strSerialNum.c_str(), nFrameNum);
    }
    else if (nMsgPointCloudType == TYPE_TEXTURED_POINT_CLOUD_WITH_NORMALS)
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclTexturedPointCloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::fromROSMsg(request.pointCloud, *pclTexturedPointCloudWithNormals);
        sprintf(chFileName, "%s/[%d]_textured_point_cloud_with_normals.pcd", g_strPointCloudSavePath.c_str(), nFrameNum);
        pcdWriter.write(chFileName, *pclTexturedPointCloudWithNormals, false);
        ROS_INFO("[Server] SN:%s save [%d]_textured_point_cloud_with_normals.pcd success", strSerialNum.c_str(), nFrameNum);
    }
    else
    {
        ROS_INFO("[Server] SN:%s point cloud type[%d] error...", strSerialNum.c_str(), nMsgPointCloudType);
        response.result.data = false;
        return true;
    }

    // 再保存图像（只处理深度和RGB）
    try
    {
        if (request.depthImg.data.size() > 0)
        {
            ROS_INFO("[Server] SN:%s Get depth image[%d] for save", strSerialNum.c_str(), nFrameNum);
            cv_bridge::CvImagePtr depthPtr = cv_bridge::toCvCopy(request.depthImg, sensor_msgs::image_encodings::TYPE_16UC1);
            char chDepthFile[256] = "";
            sprintf(chDepthFile, "%s/[%d]_DepthImage.png", g_strImageSavePath.c_str(), nFrameNum);
            cv::imwrite(chDepthFile, depthPtr->image);
            ROS_INFO("[Server] SN:%s Save [%d]_DepthImage.png success", strSerialNum.c_str(), nFrameNum);
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("[Server] SN:%s cv_bridge depth image Error! %s", strSerialNum.c_str(), e.what());
    }

    try
    {
        if (request.rgbImg.data.size() > 0)
        {
            ROS_INFO("[Server] SN:%s Get rgb image[%d] for save", strSerialNum.c_str(), nFrameNum);
            cv_bridge::CvImagePtr rgbPtr = cv_bridge::toCvCopy(request.rgbImg, sensor_msgs::image_encodings::BGR8);
            char chRgbFile[256] = "";
            sprintf(chRgbFile, "%s/[%d]_RgbImage.png", g_strImageSavePath.c_str(), nFrameNum);
            cv::imwrite(chRgbFile, rgbPtr->image);
            ROS_INFO("[Server] SN:%s Save [%d]_RgbImage.png success", strSerialNum.c_str(), nFrameNum);
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("[Server] SN:%s cv_bridge rgb image Error! %s", strSerialNum.c_str(), e.what());
    }

    // 此处暂时只使用 CameraInfo 进行日志输出，后续如需持久化可扩展
    ROS_DEBUG("[Server] depthCamInfo width:%d height:%d", request.depthCamInfo.width, request.depthCamInfo.height);
    ROS_DEBUG("[Server] rgbCamInfo   width:%d height:%d", request.rgbCamInfo.width, request.rgbCamInfo.height);

    response.result.data = true;
    return true;
}

