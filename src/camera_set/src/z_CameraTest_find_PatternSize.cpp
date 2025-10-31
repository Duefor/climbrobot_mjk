// 识别棋盘格

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 初始化 RealSense
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);

    // 棋盘格参数（内部角点数，不是方块数）
    const int board_width = 8;
    const int board_height = 6;
    cv::Size pattern_size(board_width, board_height);

    while (true) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::video_frame color_frame = frames.get_color_frame();

        int width = color_frame.get_width();
        int height = color_frame.get_height();

        cv::Mat color_img(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat gray_img;
        cv::cvtColor(color_img, gray_img, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray_img, pattern_size, corners,
                                               cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            // 亚像素优化
            cv::cornerSubPix(gray_img, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            // 画出角点
            cv::drawChessboardCorners(color_img, pattern_size, corners, found);
            std::cout << "棋盘格识别成功，角点数: " << corners.size() << std::endl;
        }

        cv::imshow("Chessboard Detection", color_img);

        if (cv::waitKey(1) == 'q') break;
    }

    return 0;
}