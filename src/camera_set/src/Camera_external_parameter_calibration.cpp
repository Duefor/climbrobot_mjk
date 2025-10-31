//相机的手眼标定(外参标定)

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;

// 读取 4x4 矩阵
cv::Mat readPoseTxt(const std::string& path) {
    std::ifstream file(path);
    cv::Mat pose = cv::Mat::eye(4, 4, CV_64F);
    for (int i = 0; i < 4 && file; ++i)
        for (int j = 0; j < 4 && file; ++j)
            file >> pose.at<double>(i, j);
    return pose;
}

int main() {
    std::string folder = "/home/duefor/climbrobot_mjk/src/camera_set/test_picture/1/";
    int count = 10;  //10组数据

    std::vector<cv::Mat> R_gripper2base, t_gripper2base;
    std::vector<cv::Mat> R_target2cam, t_target2cam;

    for (int i = 0; i < count; ++i) {
        // ==== 读取 A_i（机械臂tcp坐标位姿）==== 注意文件格式应该为a_i.txt
        cv::Mat A = readPoseTxt(folder + "a_" + std::to_string(i) + ".txt");
        cv::Mat R_A = A(cv::Rect(0, 0, 3, 3)).clone();
        cv::Mat t_A = A(cv::Rect(3, 0, 1, 3)).clone();
        R_gripper2base.push_back(R_A);
        t_gripper2base.push_back(t_A);

        // ==== 读取 B_i（棋盘格相对于相机的位姿）==== 注意文件格式应该为pose_i.txt
        cv::Mat B = readPoseTxt(folder + "pose_" + std::to_string(i) + ".txt");
        cv::Mat R_B = B(cv::Rect(0, 0, 3, 3)).clone();
        cv::Mat t_B = B(cv::Rect(3, 0, 1, 3)).clone();
        R_target2cam.push_back(R_B);
        t_target2cam.push_back(t_B);
    }

    // ==== 求解手眼变换 ====
    cv::Mat R_cam2gripper, t_cam2gripper;
    cv::calibrateHandEye(
        R_gripper2base, t_gripper2base,
        R_target2cam, t_target2cam,
        R_cam2gripper, t_cam2gripper,
        cv::CALIB_HAND_EYE_TSAI
    );

    // ==== 构造齐次变换矩阵 X ====
    cv::Mat X = cv::Mat::eye(4, 4, CV_64F);
    R_cam2gripper.copyTo(X(cv::Rect(0, 0, 3, 3)));
    t_cam2gripper.copyTo(X(cv::Rect(3, 0, 1, 3)));

    std::cout << "\n✅ 相机相对于末端执行器的变换矩阵 (X):\n" << X << std::endl;

    // 保存结果
    std::ofstream file(folder + "hand_eye_result.txt");
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            file << X.at<double>(i, j);
            if (j < 3) file << " ";
        }
        file << "\n";
    }
    file.close();
    std::cout << "📁 已保存到 hand_eye_result.txt" << std::endl;

    return 0;
}
