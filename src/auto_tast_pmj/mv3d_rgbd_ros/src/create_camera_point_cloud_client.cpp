#include <ros/ros.h>
#include <ros/package.h>

#include "mv3d_rgbd_ros/pointcloud.h"
#include "mv3d_rgbd_ros/pointcloudRequest.h"
#include "mv3d_rgbd_ros/pointcloudResponse.h"
#include "../common/hik_camera.h"

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    // 初始化
    ros::init(argc, argv, "hik_camera_point_cloud_client");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    // 获取点云类型
    int nPointCloudType = 1;
    ros::param::get(ros::this_node::getName() + "/pd_type", nPointCloudType);
    if(nPointCloudType < 1 || nPointCloudType > 4)
    {
        ROS_INFO("Error point cloud type:[%d]", nPointCloudType);
        return 0;
    }

    // 创建客户端
    ros::ServiceClient client;
    client = nh.serviceClient<mv3d_rgbd_ros::pointcloud>("SavePointCloud");

    ROS_INFO("---------------- Waiting server start ----------------");
    // 线程阻塞，等待服务器启动
    ros::service::waitForService("SavePointCloud");
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
    
    // 获取序列号
    string strCamSerialNumber = camera.getSerialNumber();
    // 设置点云输出
    MV3D_RGBD_PARAM stParam;
    stParam.enParamType = ParamType_Enum;
    stParam.ParamInfo.stEnumParam.nCurValue = nPointCloudType;
    int nRet = camera.setParam(MV3D_RGBD_ENUM_POINT_CLOUD_OUTPUT, &stParam);
    if(MV3D_RGBD_OK != nRet)
    {
        ROS_INFO("[Client] SN:%s Set point cloud type[%d] failed...", strCamSerialNumber, nPointCloudType);
        camera.close();
        return 0;
    }

    // 开始取流
    camera.start();
    mv3d_rgbd_ros::pointcloud msgs;
    bool bExit = false;
    while(!bExit && ros::ok())
    {
        // 获取点云信息
        sensor_msgs::PointCloud2 stPointCloudMsg = camera.getPointCloud();
        // 获取图像类型信息
        if(camera.getMsgPointCloudType())
        {
            stPointCloudMsg.header.frame_id = "point_cloud_map";
            // 组合消息包
            msgs.request.pointCloud = stPointCloudMsg;
            msgs.request.camSerialNum = strCamSerialNumber;
            msgs.request.frameNum = camera.getFrameNum();
            msgs.request.timeStamp = camera.getTimeStamp();
            msgs.request.typeInfo = camera.getMsgPointCloudType();
            ROS_INFO("[Client] SN:%s send point cloud[%d] msgs success", msgs.request.camSerialNum.c_str(), msgs.request.frameNum);
            // 校验是否发送成功
            bExit = !client.call(msgs); 
        }
        rate.sleep();
        ros::spinOnce();
    }
    camera.stop();
    camera.close();
    // 关闭客户端
    client.shutdown();
    ROS_INFO("Point cloud client shut down!");
    return 0;
}

