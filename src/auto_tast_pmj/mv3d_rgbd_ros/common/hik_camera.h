#ifndef HIK_CAMERA_H
#define HIK_CAMERA_H

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <camera_info_manager/camera_info_manager.h>

#include "Mv3dRgbdApi.h"
#include "Mv3dRgbdDefine.h"
#include "Mv3dRgbdImgProc.h"

#define TYPE_DEPTH                                 1 << 0
#define TYPE_RGB                                   1 << 1
#define TYPE_LEFT_IR                               1 << 2
#define TYPE_RIGHT_IR                              1 << 3

#define TYPE_POINT_CLOUD                           1
#define TYPE_POINT_CLOUD_WITH_NORMALS              2
#define TYPE_TEXTURED_POINT_CLOUD                  3
#define TYPE_TEXTURED_POINT_CLOUD_WITH_NORMALS     4

using namespace std;

class HikCamera
{
public:
    HikCamera();
    ~HikCamera();

    // 初始化函数，显示SDK版本，并枚举设备
    void initialize();
    
    // 连接设备
    void connect(const char* chSerialNumber = nullptr);

    // 开始取流
    void start();

    // 停止取流
    void stop();

    // 关闭设备
    void close();

    // 获取设备序列号
    string getSerialNumber();

    // 获取帧号
    uint32_t getFrameNum();

    // 获取时戳
    int64_t getTimeStamp();

    // 设置参数
    int setParam(const char* chParam, MV3D_RGBD_PARAM* pstParam);

    // 获取参数
    int getParam(const char* chParam, MV3D_RGBD_PARAM* pstParam);

    // 获取相机标定参数
    vector<sensor_msgs::CameraInfo> getCamInfo();
    
    // 获取相机图像指针
    vector<sensor_msgs::ImagePtr> getImage();

    // 获取消息包附加的图像类型
    uint32_t getMsgImgType();

    // 获取SDK得到的点云类型
    uint32_t getMsgPointCloudType();

    // 获取点云
    sensor_msgs::PointCloud2 getPointCloud();

    // 通过OpenCV保存图像
    bool saveImage(const char* chFileName, int enImageMsgType);

    // 通过PCL保存点云
    bool savePointCloud(const char* chFileName, int enPointCloudMsgType);

private:
    // 普通点云转换
    void convertToPCL(const MV3D_RGBD_IMAGE_DATA& stPointCloud, pcl::PointCloud<pcl::PointXYZ>& pclPointCloud);

    // 纹理点云转换
    void convertToPCL(const MV3D_RGBD_IMAGE_DATA& stPointCloud, pcl::PointCloud<pcl::PointXYZRGB>& pclPointCloud);
    
    // 法向量点云转换
    void convertToPCL(const MV3D_RGBD_IMAGE_DATA& stPointCloud, pcl::PointCloud<pcl::PointNormal>& pclPointCloud);

    // 带法向量的纹理点云转换
    void convertToPCL(const MV3D_RGBD_IMAGE_DATA& stPointCloud, pcl::PointCloud<pcl::PointXYZRGBNormal>& pclPointCloud);

private:
    // 相机句柄
    void* m_handle = nullptr;
    // 设备序列号
    string m_strSerialNum;
    // 彩色图像指针
    sensor_msgs::ImagePtr m_pRgbImg;
    // 深度图像指针
    sensor_msgs::ImagePtr m_pDepthImg;
    // 左IR图像指针
    sensor_msgs::ImagePtr m_pLeftIrImg;
    // 右IR图像指针
    sensor_msgs::ImagePtr m_pRightIrImg;
    // 消息图像类型
    uint32_t m_nMsgImgType = 0;
    // 消息点云类型
    uint32_t m_nMsgPointCloudType = 0;
    // 彩色图缓存
    unsigned char* m_pRgbBuffer = NULL;
    // 彩色图缓存长度;
    unsigned int m_nRgbBufferLen = 0;
    // 图像帧号
    uint32_t m_nFrameNum = 0;
    // 图像时戳
    int64_t m_nTimeStamp = 0;
    // OpenCV图像
    cv::Mat m_matDepthImg;
    cv::Mat m_matRgbImg;
    cv::Mat m_matLeftIrImg;
    cv::Mat m_matRightIrImg;
    // PCL点云图
    pcl::PointCloud<pcl::PointXYZ> m_pclPointCloud;
    pcl::PointCloud<pcl::PointXYZRGB> m_pclTexturedPointCloud;
    pcl::PointCloud<pcl::PointNormal> m_pclPointCloudWithNormals;
    pcl::PointCloud<pcl::PointXYZRGBNormal> m_pclTexturedPointCloudWithNormals;
};

#endif