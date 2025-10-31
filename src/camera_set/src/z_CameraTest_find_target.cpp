#include <opencv2/opencv.hpp>
#include <iostream>
int main() {
    cv::Mat img = cv::imread("/home/duefor/climbrobot_mjk/src/camera_set/test_picture/test6.jpg");
    if (img.empty()) {
        std::cerr << "图像读取失败！" << std::endl;
        return -1;
    }

    float scale = 0.3;
    cv::resize(img, img, cv::Size(), scale, scale);

    // 转 HSV
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    // 使用你提供的 HSV 范围
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(50, 0, 200), cv::Scalar(90, 30, 250), mask);

    // 膨胀，连通贴纸
    cv::Mat morph;
    cv::dilate(mask, morph, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(morph, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int count = 0;
    for (const auto& contour : contours) {
        cv::Rect box = cv::boundingRect(contour);
        if (box.width < 15 || box.height < 15 || box.area() > 10000) continue;

        cv::rectangle(img, box, cv::Scalar(0, 255, 255), 2);
        ++count;
    }

    std::cout << "检测到贴纸数量：" << count << std::endl;
    cv::imshow("贴纸区域检测结果", img);
    cv::waitKey(0);
    return 0;
}
