// 相机内参标定 获取数据集

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>  // C++17



int main() {
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);

    // 棋盘格参数
    const int board_width = 8;
    const int board_height = 6;
    const float square_size = 0.014f; // 14mm
    cv::Size pattern_size(board_width, board_height);
    int image_index = 0;

    // 准备 3D 角点坐标（只计算一次）
    std::vector<cv::Point3f> object_corners;
    for (int i = 0; i < board_height; ++i)
        for (int j = 0; j < board_width; ++j)
            object_corners.emplace_back(j * square_size, i * square_size, 0);


    namespace fs = std::filesystem;

    std::string save_dir = "/home/duefor/climbrobot_mjk/src/camera_set/test_picture/1/";

    // 创建目录（如果不存在）
    if (!fs::exists(save_dir)) {
        fs::create_directories(save_dir);
    }


    while (true) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::video_frame color_frame = frames.get_color_frame();

        int width = color_frame.get_width();
        int height = color_frame.get_height();
        cv::Mat color_img(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat gray;
        cv::cvtColor(color_img, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, pattern_size, corners);

        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            cv::drawChessboardCorners(color_img, pattern_size, corners, found);
        }

        cv::imshow("Capture Chessboard", color_img);
        char key = cv::waitKey(1);

        if (key == 'q') break;

        if (found && key == 's') {
            std::string img_name = save_dir + "calib_image_" + std::to_string(image_index) + ".png";
            std::string txt_name = save_dir + "points_" + std::to_string(image_index) + ".txt";

            // 保存图像
            cv::imwrite(img_name, color_img);
            std::cout << "✅ 保存图像: " << img_name << std::endl;

            // 保存角点（2D + 3D）到 txt 文件
            std::ofstream file(txt_name);
            for (size_t i = 0; i < corners.size(); ++i) {
                file << corners[i].x << " " << corners[i].y << " "
                     << object_corners[i].x << " "
                     << object_corners[i].y << " "
                     << object_corners[i].z << std::endl;
            }
            file.close();
            std::cout << "✅ 保存角点: " << txt_name << std::endl;

            image_index++;
        }
    }

    return 0;
}
