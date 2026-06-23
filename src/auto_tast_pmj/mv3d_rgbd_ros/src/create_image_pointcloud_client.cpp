#include <ros/ros.h>
#include <iostream>

#include "mv3d_rgbd_ros/image_pointcloud.h"
#include "mv3d_rgbd_ros/image_pointcloudRequest.h"
#include "mv3d_rgbd_ros/image_pointcloudResponse.h"

#include "../common/hik_camera.h"

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    // 初始化节点
    ros::init(argc, argv, "hik_camera_image_pointcloud_client");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    // 获取点云类型（1~4）
    int nPointCloudType = 1;
    ros::param::get(ros::this_node::getName() + "/pd_type", nPointCloudType);
    if (nPointCloudType < 1 || nPointCloudType > 4)
    {
        ROS_INFO("Error point cloud type:[%d]", nPointCloudType);
        return 0;
    }

    ros::ServiceClient client;
    // 创建一个客户端句柄，指向 SaveImagePointCloud 服务
    client = nh.serviceClient<mv3d_rgbd_ros::image_pointcloud>("SaveImagePointCloud"); 

    ROS_INFO("---------------- Waiting SaveImagePointCloud server start ----------------");
    // 线程阻塞，等待服务器启动
    ros::service::waitForService("SaveImagePointCloud");

    HikCamera camera;
    camera.initialize();

    // 获取序列号
    std::string strSerialNumber = "None";
    ros::param::get(ros::this_node::getName() + "/cam_sn", strSerialNumber);
    if (strSerialNumber.compare("None") == 0)
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

    // 获取实际连接到的序列号
    std::string strCamSerialNumber = camera.getSerialNumber();

    // 设置点云输出类型
    MV3D_RGBD_PARAM stParam;
    stParam.enParamType = ParamType_Enum;
    stParam.ParamInfo.stEnumParam.nCurValue = nPointCloudType;
    int nRet = camera.setParam(MV3D_RGBD_ENUM_POINT_CLOUD_OUTPUT, &stParam);
    if (MV3D_RGBD_OK != nRet)
    {
        ROS_INFO("[Client] SN:%s Set point cloud type[%d] failed...", strCamSerialNumber.c_str(), nPointCloudType);
        camera.close();
        return 0;
    }

    // 获取标定信息
    std::vector<sensor_msgs::CameraInfo> stCamInfo = camera.getCamInfo();

    // 开始取流
    camera.start();
    mv3d_rgbd_ros::image_pointcloud msgs;

    // 循环等待用户按回车键，每次按下抓取并发送一帧
    while (ros::ok())
    {
        std::cout << "Press Enter to capture image + point cloud and send once (Ctrl+C to exit)..." << std::endl;
        std::cin.get();


        // 获取图像指针信息
        std::vector<sensor_msgs::ImagePtr> stImagePtr = camera.getImage();
        // 获取图像类型信息（位掩码）
        uint32_t nImgType = camera.getMsgImgType();

        // 获取点云信息
        sensor_msgs::PointCloud2 stPointCloudMsg = camera.getPointCloud();
        uint32_t nPointCloudMsgType = camera.getMsgPointCloudType();

        if (nPointCloudMsgType)
        {
            stPointCloudMsg.header.frame_id = "point_cloud_map";

            // 组合消息包
            msgs.request.camSerialNum = strCamSerialNumber;
            msgs.request.typeInfo = nPointCloudMsgType;
            msgs.request.frameNum = camera.getFrameNum();
            msgs.request.timeStamp = camera.getTimeStamp();
            msgs.request.pointCloud = stPointCloudMsg;

            // 深度 / 彩色图像及相机内参
            if (nImgType & TYPE_DEPTH)
            {
                msgs.request.depthImg = *stImagePtr[0];
                msgs.request.depthCamInfo = stCamInfo[0];
            }
            if (nImgType & TYPE_RGB)
            {
                msgs.request.rgbImg = *stImagePtr[1];
                msgs.request.rgbCamInfo = stCamInfo[1];
            }

            ROS_INFO("[Client] SN:%s send image+point cloud[%d] msgs success", msgs.request.camSerialNum.c_str(), msgs.request.frameNum);

            // 校验是否发送成功，
            // call调用，请求-应答，将msgs.request发送给服务端，等待server回调函数执行完，填好msgs.response，然后返回 true/false。
            if (!client.call(msgs))
            {
                ROS_ERROR("[Client] SN:%s call service failed", msgs.request.camSerialNum.c_str());
            }
        }

        else
        {
            ROS_WARN("[Client] SN:%s no valid point cloud type for current frame", strCamSerialNumber.c_str());
        }
    }

    camera.stop();
    camera.close();

    // 关闭客户端
    client.shutdown();
    ROS_INFO("Image point cloud client shut down!");

    return 0;
}

