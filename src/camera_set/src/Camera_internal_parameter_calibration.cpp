// 相机内参标定，获得内参矩阵

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;

int main() {
    std::string data_dir = "/home/duefor/climbrobot_mjk/src/camera_set/test_picture/1/";
    int num_images = 11;  // 根据你采了多少组数据

    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;
    cv::Size image_size;

    for (int i = 0; i < num_images; ++i) {
        std::string img_path = data_dir + "calib_image_" + std::to_string(i) + ".png";
        std::string txt_path = data_dir + "points_" + std::to_string(i) + ".txt";

        // 读取图像
        cv::Mat img = cv::imread(img_path);
        if (img.empty()) {
            std::cerr << "读取图像失败: " << img_path << std::endl;
            continue;
        }

        // 获取图像尺寸
        if (image_size.width == 0)
            image_size = img.size();

        // 读取角点坐标
        std::vector<cv::Point2f> img_pts;
        std::vector<cv::Point3f> obj_pts;
        std::ifstream file(txt_path);
        if (!file.is_open()) {
            std::cerr << "打开点文件失败: " << txt_path << std::endl;
            continue;
        }

        float x, y, X, Y, Z;
        while (file >> x >> y >> X >> Y >> Z) {
            img_pts.emplace_back(x, y);
            obj_pts.emplace_back(X, Y, Z);
        }

        image_points.push_back(img_pts);
        object_points.push_back(obj_pts);
    }

    // 标定
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    double error = cv::calibrateCamera(object_points, image_points, image_size,
                                       cameraMatrix, distCoeffs, rvecs, tvecs);

    // 打印结果
    std::cout << "✅ 相机内参矩阵:\n" << cameraMatrix << std::endl;
    std::cout << "✅ 畸变系数:\n" << distCoeffs.t() << std::endl;
    std::cout << "✅ 重投影误差: " << error << std::endl;

    // 保存结果到 XML
    std::string output_file = data_dir + "calibration_result.xml";
    cv::FileStorage fs(output_file, cv::FileStorage::WRITE);
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs.release();
    std::cout << "📁 已保存标定结果到: " << output_file << std::endl;

    return 0;
}
