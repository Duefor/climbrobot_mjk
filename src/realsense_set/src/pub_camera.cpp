#include <Eigen/Core>
#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "camera_and_point_cloud_publisher");

    // 创建节点句柄
    ros::NodeHandle nh;

   // 图像话题发布者，使用image_transport库来创建合适的图像发布者
    image_transport::ImageTransport it(nh);
    image_transport::Publisher depth_image_pub = it.advertise("/camera/depth/image_raw", 1);
    image_transport::Publisher color_image_pub = it.advertise("/camera/color/image_raw", 1);

    // 点云话题发布者
    ros::Publisher point_cloud_pub;
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud2cam", 1);
    // 相机内参话题发布者
    ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/color/camera_info", 1);
    // 发布相机imu数据
    ros::Publisher cameraimu_pub = nh.advertise<sensor_msgs::Imu>("/camera/imu", 1);

    std::cout<<"正在初始化相机..."<<std::endl;
    rs2::pipeline pipe;

    // 配置相机参数
    rs2::config cfg;
    cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, 1280, 720, rs2_format::RS2_FORMAT_Z16, 30);
    cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, 1280, 720, rs2_format::RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_GYRO);

    // 启动管道
    rs2::pipeline_profile profile = pipe.start(cfg);

    rs2_stream align_to = RS2_STREAM_COLOR;
    rs2::align align(align_to);
    std::cout<<"初始化完成"<<std::endl;

    // 循环发布数据
    ros::Rate rate(30); // 设置发布频率为30Hz，可根据实际需求调整
    int count = 0;
    while (ros::ok()) {
        ros::Time timestamp = ros::Time::now(); // 保证同一时间戳

        double usedtime;
        double start = ros::Time::now().toSec();
        // 获取一帧数据
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        rs2::video_frame color_frame = frames.get_color_frame();
        rs2::frameset aligned_frames = align.process(frames);
        rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();
        rs2::video_frame aligned_color_frame = aligned_frames.get_color_frame();
        rs2::frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
        rs2::frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);
        if (!aligned_frames) continue;

        // 相机imu数据获取
        rs2::motion_frame accel_motion = accel_frame.as<rs2::motion_frame>();
        rs2::motion_frame gyro_motion = gyro_frame.as<rs2::motion_frame>();
        rs2_vector accel_data = accel_motion.get_motion_data();
        rs2_vector gyro_data = gyro_motion.get_motion_data();

        // 填充sensor_msgs::Imu消息
        sensor_msgs::Imu imu_msg;

        // 填充线加速度数据，注意要将rs2_vector类型的数据转换为符合sensor_msgs::Imu消息要求的类型
        imu_msg.linear_acceleration.x = accel_data.x;
        imu_msg.linear_acceleration.y = accel_data.y;
        imu_msg.linear_acceleration.z = accel_data.z;
        // std::cout << "accel_data: " << accel_data.x << " " << accel_data.y << " " << accel_data.z << std::endl;

        // 填充角速度数据
        imu_msg.angular_velocity.x = gyro_data.x;
        imu_msg.angular_velocity.y = gyro_data.y;
        imu_msg.angular_velocity.z = gyro_data.z;

        // 将深度图像转换为8位图像并应用颜色映射
        cv::Mat aligned_depth_image(720, 1280, CV_16U, (void*)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
        // 创建一个CvBridge对象
        cv_bridge::CvImage cv_depth;
        // 将OpenCV图像转换为ROS图像消息，这里不进行一系列变换，直接发布保证深度图的精度
        cv_depth.image = aligned_depth_image;
        cv_depth.encoding = sensor_msgs::image_encodings::TYPE_16UC1;

        // 将彩色帧转换为OpenCV格式
        cv::Mat aligned_color_image(720, 1280, CV_8UC3, (void*)aligned_color_frame.get_data(), cv::Mat::AUTO_STEP);
        // 创建一个CvBridge对象
        cv_bridge::CvImage cv_color;
        // 将OpenCV图像转换为ROS图像消息
        cv_color.image = aligned_color_image;
        cv_color.encoding = "bgr8";

        // 保存获取相应数据时的时间戳，保证时间同步
        cv_depth.header.stamp = timestamp;
        cv_depth.header.frame_id = "camera_depth_optical_frame";
        auto depth_image_msg = cv_depth.toImageMsg();
        cv_color.header.stamp = timestamp;
        cv_color.header.frame_id = "camera_color_optical_frame";
        auto color_image_msg = cv_color.toImageMsg();
        imu_msg.header.stamp = timestamp;  // 设置消息的时间戳为当前时间
        imu_msg.header.frame_id = "camera_link";  // 根据实际情况设置合适的坐标系名称

        // 发布深度图像
        depth_image_pub.publish(depth_image_msg);
        // 发布彩色图像
        color_image_pub.publish(color_image_msg);
        // 发布imu消息
        cameraimu_pub.publish(imu_msg);

        // 获取深度图相机内参
        rs2::video_stream_profile depth_intrin = aligned_depth_frame.get_profile().as<rs2::video_stream_profile>();
        const auto depth_intrinsics = depth_intrin.get_intrinsics();

        // 填充相机内参消息
        sensor_msgs::CameraInfo camera_info_msg;
        camera_info_msg.header.frame_id = "camera_color_optical_frame";
        camera_info_msg.header.stamp = timestamp;

        // 设置相机内参
        camera_info_msg.width = depth_intrin.get_intrinsics().width;
        camera_info_msg.height = depth_intrin.get_intrinsics().height;
        camera_info_msg.distortion_model = "plumb_bob";

        // 内参矩阵
        camera_info_msg.K[0] = depth_intrin.get_intrinsics().fx;
        camera_info_msg.K[1] = 0;
        camera_info_msg.K[2] = depth_intrin.get_intrinsics().ppx;
        camera_info_msg.K[3] = 0;
        camera_info_msg.K[4] = depth_intrin.get_intrinsics().fy;
        camera_info_msg.K[5] = depth_intrin.get_intrinsics().ppy;
        camera_info_msg.K[6] = 0;
        camera_info_msg.K[7] = 0;
        camera_info_msg.K[8] = 1;

        // 畸变系数，这里假设为0（如果有实际畸变系数，可替换）
        camera_info_msg.D.push_back(0);
        camera_info_msg.D.push_back(0);
        camera_info_msg.D.push_back(0);
        camera_info_msg.D.push_back(0);
        camera_info_msg.D.push_back(0);

        // 发布相机内参消息
        camera_info_pub.publish(camera_info_msg);
        
        std::cout << "发布图像和点云数据中..." << std::endl;
        count++;
        if (count > 1000) count = 0;

        usedtime = ros::Time::now().toSec() - start;
        std::cout << "程序运行时间：" << usedtime << "s" << std::endl;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}