#include "../common/hik_camera.h"
#include "mv3d_rgbd_ros/image.h"
#include "mv3d_rgbd_ros/imageRequest.h"
#include "mv3d_rgbd_ros/imageResponse.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    // 初始化
    ros::init(argc, argv, "hik_camera_image_client");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    // 创建客户端
    ros::ServiceClient client;
    client = nh.serviceClient<mv3d_rgbd_ros::image>("SaveImage");

    ROS_INFO("---------------- Waiting server start ----------------");
    // 线程阻塞，等待服务器启动
    ros::service::waitForService("SaveImage");
    HikCamera camera; 
    camera.initialize();

    // 获取序列号
    string strSerialNumber = "None";
    ros::param::get(ros::this_node::getName() + "/cam_sn", strSerialNumber);
    if(strSerialNumber.compare("None") == 0)
    {
        // 默认连接第一个相机
        camera.connect();
    }
    else
    {
        ROS_INFO("---------------- Connect by serial number: %s ----------------", strSerialNumber.c_str());
        // 根据序列号进行连接
        camera.connect(strSerialNumber.c_str());
    }

    // 获取标定信息
    vector<sensor_msgs::CameraInfo> stCamInfo = camera.getCamInfo();

    // 开始取流
    camera.start();
    mv3d_rgbd_ros::image msgs;
    bool bExit = false;
    while(!bExit && ros::ok())
    {
        // 获取图像指针信息
        vector<sensor_msgs::ImagePtr> stImagePtr = camera.getImage();
        // 获取图像类型信息
        uint32_t nImgType = camera.getMsgImgType();
        // 组合消息包
        msgs.request.camSerialNum = camera.getSerialNumber();
        msgs.request.depthCamInfo = stCamInfo[0];
        msgs.request.rgbCamInfo = stCamInfo[1];
        msgs.request.frameNum = camera.getFrameNum();
        msgs.request.timeStamp = camera.getTimeStamp();
        msgs.request.typeInfo = camera.getMsgImgType();
        if(msgs.request.typeInfo & TYPE_DEPTH)
        {
            msgs.request.depthImg = *stImagePtr[0];
        }
        if(msgs.request.typeInfo & TYPE_RGB)
        {
            msgs.request.rgbImg = *stImagePtr[1];
        }
        if(msgs.request.typeInfo & TYPE_LEFT_IR)
        {
            msgs.request.leftIrImg = *stImagePtr[2];
        }
        if(msgs.request.typeInfo & TYPE_RIGHT_IR)
        {
            msgs.request.rightIrImg = *stImagePtr[3];
        }
        if(msgs.request.typeInfo)
        {
            ROS_INFO("[Client] SN:%s Send image[%d] msgs success", msgs.request.camSerialNum.c_str(), msgs.request.frameNum);
        }
        // 校验是否发送成功
        bExit = !client.call(msgs); 
        rate.sleep();
        ros::spinOnce();
    }
    camera.stop();
    camera.close();
    return 0;
    

}