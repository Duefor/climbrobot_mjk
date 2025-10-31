// 显示彩色图像

#include <librealsense2/rs.hpp>         // RealSense SDK
#include <opencv2/opencv.hpp>          // OpenCV
#include <iostream>

int main() {
    // 创建 RealSense 管道
    rs2::pipeline pipe;
    rs2::config cfg;

    // 启用彩色流
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    // 启动流
    pipe.start(cfg);

    while (true) {
        // 等待帧
        rs2::frameset frames = pipe.wait_for_frames();

        // 获取彩色帧
        rs2::frame color_frame = frames.get_color_frame();

        // 获取图像宽高
        int width = color_frame.as<rs2::video_frame>().get_width();
        int height = color_frame.as<rs2::video_frame>().get_height();

        // 转换为 OpenCV 格式
        cv::Mat color_img(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // 显示图像
        cv::imshow("RealSense Color", color_img);

        // 按下 'q' 键退出
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}