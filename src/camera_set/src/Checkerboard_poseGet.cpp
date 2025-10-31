// 实时批量处理多张棋盘格图像，并输出每张图像中的棋盘格及其在相机坐标系下的位姿变换矩阵

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;

int main() {
    // 相机内参和畸变参数
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        6.0158229453345143e+02, 0, 3.0796418819972678e+02,
        0, 6.0560382659808170e+02, 2.2459305154748191e+02,
        0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 3.9197556074161991e-02, 7.4904446156167093e-01, -2.3009517470044552e-02, 4.4836709900289724e-03, -2.8847758499111831e+00);

    //棋盘格参数
    const int board_width = 8;
    const int board_height = 6;
    const float square_size = 0.014f;  //14mm
    cv::Size pattern_size(board_width, board_height);

    // ==== 保存路径 ====
    std::string save_dir = "/home/duefor/climbrobot_mjk/src/camera_set/test_picture/1/";
    fs::create_directories(save_dir);
    int index = 0;

    // ==== 启动 RealSense 相机 ====
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    pipe.start(cfg);

    std::cout << "按 's' 拍照 + 标定并保存 B_i，按 'q' 退出" << std::endl;

    while (true) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::video_frame color_frame = frames.get_color_frame();

        int w = color_frame.get_width();
        int h = color_frame.get_height();
        cv::Mat color(cv::Size(w, h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat display = color.clone();

        // 实时检测角点
        std::vector<cv::Point2f> image_points;
        bool found = cv::findChessboardCorners(color, pattern_size, image_points);
        if (found) {
            cv::Mat gray;
            cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(gray, image_points, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            cv::drawChessboardCorners(display, pattern_size, image_points, found);
        }

        // 显示图像
        cv::imshow("Live View", display);
        char key = cv::waitKey(1);
        if (key == 'q') break;

        if (key == 's' && found) {
            // ====== 保存图像 ======
            std::string img_path = save_dir + "calib_image_" + std::to_string(index) + ".png";
            std::string pose_path = save_dir + "pose_" + std::to_string(index) + ".txt";
            cv::imwrite(img_path, color);
            std::cout << "保存图像: " << img_path << std::endl;

            // ====== 构造 3D 角点坐标 ======
            std::vector<cv::Point3f> object_points;
            for (int y = 0; y < board_height; ++y)
                for (int x = 0; x < board_width; ++x)
                    object_points.emplace_back(x * square_size, y * square_size, 0);

            // ====== 求解位姿 B_i ======
            cv::Mat rvec, tvec;
            cv::solvePnP(object_points, image_points, cameraMatrix, distCoeffs, rvec, tvec);
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            cv::Mat pose = cv::Mat::eye(4, 4, CV_64F);
            R.copyTo(pose(cv::Rect(0, 0, 3, 3)));
            tvec.copyTo(pose(cv::Rect(3, 0, 1, 3)));

            std::cout << "B_" << index << " = \n" << pose << std::endl;

            // ====== 保存位姿矩阵 ======
            std::ofstream file(pose_path);
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    file << pose.at<double>(r, c);
                    if (c < 3) file << " ";
                }
                file << "\n";
            }
            file.close();
            std::cout << "已保存位姿矩阵: " << pose_path << std::endl;

            index++;
        }
    }

    pipe.stop();
    return 0;
}
