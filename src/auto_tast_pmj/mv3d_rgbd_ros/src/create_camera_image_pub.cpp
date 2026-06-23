#include "../common/hik_camera.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hik_camera_image_publisher");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    ros::Publisher depthPub = nh.advertise<sensor_msgs::Image>("/camera/depth", 1000);
    ros::Publisher rgbPub = nh.advertise<sensor_msgs::Image>("/camera/rgb", 1000);
    ros::Publisher leftIrPub = nh.advertise<sensor_msgs::Image>("/camera/leftIr", 1000);
    ros::Publisher rightIrPub = nh.advertise<sensor_msgs::Image>("/camera/rightIr", 1000);
    ros::Publisher depthCamInfoPub = nh.advertise<sensor_msgs::CameraInfo>("/depthCamInfo", 1000);
    ros::Publisher rgbCamInfoPub = nh.advertise<sensor_msgs::CameraInfo>("/rgbCamInfo", 1000);

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

    //定义Frame ID
    std::string rgb_frame_id = "camera_color_optical_frame";
    std::string depth_frame_id = "camera_depth_optical_frame";

    int nSeq = 0;
    while(ros::ok())
    {
        //获取当前时刻
        ros::Time current_time = ros::Time::now();
        // 获取图像指针信息
        vector<sensor_msgs::ImagePtr> stImagePtr = camera.getImage();
        // 获取图像类型信息
        uint32_t nImgType = camera.getMsgImgType();
        // 发布消息
        if(nImgType & TYPE_DEPTH)
        {
            // 填充 Header
            stImagePtr[0]->header.seq = nSeq;
            stImagePtr[0]->header.stamp = current_time;
            stImagePtr[0]->header.frame_id = depth_frame_id; // 深度图坐标系
            depthPub.publish(*stImagePtr[0]);
            // ROS_INFO("[Publisher] Send depth image [%d] success", nSeq);
        }
        if(nImgType & TYPE_RGB)
        {
            stImagePtr[1]->header.seq = nSeq;
            stImagePtr[1]->header.stamp = current_time;
            stImagePtr[1]->header.frame_id = rgb_frame_id;

            rgbPub.publish(*stImagePtr[1]);
            // ROS_INFO("[Publisher] Send rgb image [%d] success", nSeq);
        }
        if(nImgType & TYPE_LEFT_IR)
        {
            leftIrPub.publish(*stImagePtr[2]);
             // ROS_INFO("[Publisher] Send left ir image [%d] success", nSeq);
        }
        if(nImgType & TYPE_RIGHT_IR)
        {
            rightIrPub.publish(*stImagePtr[3]);
            // ROS_INFO("[Publisher] Send right ir image [%d] success", nSeq);
        }
        depthCamInfoPub.publish(stCamInfo[0]);
        if (stCamInfo.size() > 1) 
            {
                stCamInfo[1].header.seq = nSeq;
                stCamInfo[1].header.stamp = current_time;    // <--- 关键修改：必须与 Image 同步
                stCamInfo[1].header.frame_id = rgb_frame_id; // <--- 关键修改：必须设置 Frame ID
                
                rgbCamInfoPub.publish(stCamInfo[1]); 
            }
        nSeq++;
        // 处理回调函数
        ros::spinOnce();
        rate.sleep();
    }
    camera.stop();
    camera.close();
    return 0;


}