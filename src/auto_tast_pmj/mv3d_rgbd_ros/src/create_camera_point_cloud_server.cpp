#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>

#include "../common/hik_camera.h"
#include "mv3d_rgbd_ros/pointcloud.h"
#include "mv3d_rgbd_ros/pointcloudRequest.h"
#include "mv3d_rgbd_ros/pointcloudResponse.h"

using namespace std;

string strSavePath;
bool SavePointCloud(mv3d_rgbd_ros::pointcloud::Request &request, mv3d_rgbd_ros::pointcloud::Response &response);

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 初始化ros节点
    ros::init(argc, argv, "hik_camera_point_cloud_server");
    ros::NodeHandle nh;
    // 获取存点云路径
    mkdir("point_cloud", S_IRWXU);
    strSavePath = "point_cloud/";
    ROS_INFO("[Server] save path: %s", strSavePath.c_str());
    // 建立服务器
    ros::ServiceServer server;
    ROS_INFO("---------------- Server start ----------------");
    server = nh.advertiseService("SavePointCloud", SavePointCloud);
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}

bool SavePointCloud(mv3d_rgbd_ros::pointcloud::Request &request, mv3d_rgbd_ros::pointcloud::Response &response)
{
    // 获取帧号
    uint32_t nFrameNum = request.frameNum;
    // 解析图像类型
    uint32_t nMsgPointCloudType = request.typeInfo;
    // 获取相机序列号
    string strSerialNum = request.camSerialNum;
    pcl::PCDWriter pcdWriter;
    char chFileName[256] = "";
    if(nMsgPointCloudType == TYPE_POINT_CLOUD)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
        // 把msg消息转化为点云
        pcl::fromROSMsg(request.pointCloud, *pclPointCloud);
        sprintf(chFileName, "%s/[%d]_point_cloud.pcd", strSavePath.c_str(), nFrameNum);
        pcdWriter.write(chFileName, *pclPointCloud, false);
        ROS_INFO("[Server] SN:%s save [%d]_point_cloud.pcd success", strSerialNum.c_str(), nFrameNum);
    }
    else if(nMsgPointCloudType == TYPE_POINT_CLOUD_WITH_NORMALS)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr pclPointCloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
        // 把msg消息转化为法向量点云
        pcl::fromROSMsg(request.pointCloud, *pclPointCloudWithNormals);
        sprintf(chFileName, "%s/[%d]_point_cloud_with_normals.pcd", strSavePath.c_str(), nFrameNum);
        pcdWriter.write(chFileName, *pclPointCloudWithNormals, false);
        ROS_INFO("[Server] SN:%s save [%d]_point_cloud_with_normals.pcd success", strSerialNum.c_str(), nFrameNum);
    }
    else if(nMsgPointCloudType == TYPE_TEXTURED_POINT_CLOUD)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclTexturedPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // 把msg消息转化为纹理点云
        pcl::fromROSMsg(request.pointCloud, *pclTexturedPointCloud);
        sprintf(chFileName, "%s/[%d]_textured_point_cloud.pcd", strSavePath.c_str(), nFrameNum);
        pcdWriter.write(chFileName, *pclTexturedPointCloud, false);
        ROS_INFO("[Server] SN:%s save [%d]_textured_point_cloud.pcd success", strSerialNum.c_str(), nFrameNum);
    }
    else if(nMsgPointCloudType == TYPE_TEXTURED_POINT_CLOUD_WITH_NORMALS)
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclTexturedPointCloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        // 把msg消息转化为带法向量的纹理点云
        pcl::fromROSMsg(request.pointCloud, *pclTexturedPointCloudWithNormals);
        sprintf(chFileName, "%s/[%d]_textured_point_cloud_with_normals.pcd", strSavePath.c_str(), nFrameNum);
        pcdWriter.write(chFileName, *pclTexturedPointCloudWithNormals, false);
        ROS_INFO("[Server] SN:%s save [%d]_textured_point_cloud_with_normals.pcd success", strSerialNum.c_str(), nFrameNum);
    }
    else
    {
        ROS_INFO("[Server] SN:%s point cloud type[%d] error...", nMsgPointCloudType);
        return false;
    }
    return true;
}