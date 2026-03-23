#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

#include <robot_sdk_wrapper/robot_sdk.h>
#include <Eigen/Dense>

// DH 参数
const double d[6]     = {0.1625, 0, 0, 0.1475, 0.0965, 0.092};
const double a[6]     = {0, 0, -0.427, -0.3905, 0, 0};
const double alpha[6] = {0, M_PI/2, 0, 0, M_PI/2, -M_PI/2};


int count_num = 0;
std::string image_save_path = "./collect_data_in/";


Eigen::Matrix4d mdhTransform(double a, double alpha, double d, double theta)
{
    double ca = cos(alpha);
    double sa = sin(alpha);
    double ct = cos(theta);
    double st = sin(theta);

    Eigen::Matrix4d T;
    // T << ct, -st*ca,  st*sa, a*ct,
    //      st,  ct*ca, -ct*sa, a*st,
    //       0,      sa,     ca,    d,
    //       0,       0,      0,    1;

    T << ct, -st, 0, a,
         st*ca, ct*ca, -sa, -sa*d,
         st*sa, ct*sa, ca,  ca*d,
         0, 0, 0, 1;


    return T;
}

// 正运动学
Eigen::Matrix4d forwardKinematics(const std::vector<double>& q)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 5; ++i)
    {
        T *= mdhTransform(a[i], alpha[i], d[i], q[i]);
    }
    return T;
}

// ======= 这里替换成你的机械臂SDK =======
std::vector<double> getRobotPose(EliteCSRobotSDK& robot)
{
    auto joint = robot.getCurrentJoint();

    std::vector<double> q(joint.begin(), joint.end());

    Eigen::Matrix4d T = forwardKinematics(q);

    Eigen::Vector3d t = T.block<3,1>(0,3);
    Eigen::Matrix3d R = T.block<3,3>(0,0);

    // 转欧拉角（ZYX）
    Eigen::Vector3d euler = R.eulerAngles(2, 1, 0); // yaw pitch roll

    return {
        t(0), t(1), t(2),
        euler(2), euler(1), euler(0)  // roll pitch yaw
    };
}
// =====================================

int main()
{
    EliteCSRobotSDK robot("192.168.1.199","192.168.1.199",true,"external_control.script","output_recipe.txt","input_recipe.txt");
    if (!robot.init())
    {
        std::cerr << "Robot init/start failed" << std::endl;
        return 1;
    }

    // 创建保存目录（如果不存在）
    system(("mkdir -p " + image_save_path).c_str());

    // 初始化 RealSense
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    pipeline.start(config);

    cv::namedWindow("detection", cv::WINDOW_NORMAL);

    while (true)
    {
        rs2::frameset frames = pipeline.wait_for_frames();
        rs2::video_frame color_frame = frames.get_color_frame();

        if (!color_frame)
            continue;

        // 转 OpenCV
        cv::Mat color_image(
            cv::Size(1280, 720),
            CV_8UC3,
            (void *)color_frame.get_data(),
            cv::Mat::AUTO_STEP);

        cv::imshow("detection", color_image);

        char key = cv::waitKey(1);

        if (key == 's')
        {
            std::cout << "采集第 " << count_num << " 组数据..." << std::endl;

            // ===== 获取机械臂位姿 =====
            std::vector<double> pose = getRobotPose(robot);

            std::cout << "机械臂pose: ";
            for (auto &p : pose)
                std::cout << p << " ";
            std::cout << std::endl;

            // ===== 保存位姿 =====
            std::ofstream file(image_save_path + "poses.txt", std::ios::app);
            for (size_t i = 0; i < pose.size(); i++)
            {
                file << pose[i];
                if (i != pose.size() - 1)
                    file << ",";
            }
            file << std::endl;
            file.close();

            // ===== 保存图像 =====
            std::string filename = image_save_path + std::to_string(count_num) + ".jpg";
            cv::imwrite(filename, color_image);

            count_num++;
        }

        // 按 q 退出
        if (key == 'q')
            break;
    }

    pipeline.stop();
    return 0;
}