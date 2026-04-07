#include <ros/ros.h> // ROS的主头文件
#include <sensor_msgs/PointCloud2.h> // ROS 点云消息类型
#include <pcl_conversions/pcl_conversions.h> // PCL和ROS点云消息类型转换
#include <pcl/point_types.h> // PCL点类型定义
#include <pcl/filters/statistical_outlier_removal.h> // 统计离群点滤波器
#include <pcl/filters/voxel_grid.h> // 体素滤波器 (下采样)
#include <pcl/filters/passthrough.h> // 直通滤波器
#include <pcl/surface/mls.h> // 移动最小二乘法表面重建
#include <pcl/features/normal_3d.h> // 法向量估计
#include <pcl/kdtree/kdtree_flann.h> // KdTree最近邻搜索
#include <pcl/visualization/pcl_visualizer.h> // PCL可视化
#include <thread> // 线程库

typedef pcl::PointXYZ PointT; // 简化点类型命名

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; // 全局PCL可视化对象
pcl::PointCloud<PointT>::Ptr global_cloud(new pcl::PointCloud<PointT>()); // 全局点云，供可视化用
pcl::PointCloud<pcl::Normal>::Ptr global_normals(new pcl::PointCloud<pcl::Normal>()); // 全局法向量，供可视化用

// 可视化线程函数，实时刷新、渲染点云与法向量
void visualizeCloud()
{
    viewer->setBackgroundColor(0, 0, 0); // 设置背景为黑色
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color(global_cloud, 0, 255, 0); // 绿色点云
    viewer->addPointCloud<PointT>(global_cloud, cloud_color, "filtered cloud"); // 添加处理后的点云
    viewer->addPointCloudNormals<PointT, pcl::Normal>(global_cloud, global_normals, 20, 0.03, "normals"); // 添加法向量，步长20，长度0.03
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "filtered cloud"); // 设置点云大小
    viewer->spin(); // 开始可视化
}

// ROS回调：接收点云消息，处理点云
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // 1. 消息转换: ROS消息转为PCL点云格式
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*msg, *cloud);

    // 2. 去噪: 统计离群点去除
    pcl::PointCloud<PointT>::Ptr cloud_denoised(new pcl::PointCloud<PointT>());
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);            // 设置用于平均的临近点数
    sor.setStddevMulThresh(1.0); // 设置标准差倍数阈值
    sor.filter(*cloud_denoised); // 滤波输出

    // 3. 滤波: 直通滤波，仅保留z轴在合适范围的点
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_denoised);
    pass.setFilterFieldName("z");   // 设置滤波字段z
    pass.setFilterLimits(0.0, 2.0); // 设置取值范围（可调整高度范围）
    pass.filter(*cloud_filtered);   // 滤波输出

    // 4. 下采样: 使用体素滤波器降低点数
    pcl::PointCloud<PointT>::Ptr cloud_downsampled(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud_filtered);
    vg.setLeafSize(0.01f, 0.01f, 0.01f); // 设置体素大小（可调，越小分辨率越高）
    vg.filter(*cloud_downsampled);

    // 5. 移动最小二乘法: 表面重建，消除点云“波浪”误差
    pcl::PointCloud<PointT>::Ptr mls_points(new pcl::PointCloud<PointT>());
    pcl::MovingLeastSquares<PointT, PointT> mls;
    mls.setInputCloud(cloud_downsampled);
    mls.setPolynomialFit(true); // 使用多项式曲面局部拟合
    mls.setSearchMethod(pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>())); // 指定搜索树
    mls.setSearchRadius(0.03); // 设置搜索半径（可调整，影响平滑程度）
    mls.process(*mls_points);

    // 6. 法向量估计: 用于点云表面与法线建模
    pcl::NormalEstimation<PointT, pcl::Normal> ne; // 新建法向量估计对象
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>()); // 构建搜索树
    tree->setInputCloud(mls_points);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    ne.setInputCloud(mls_points);
    ne.setSearchMethod(tree); // 设置搜索方法
    ne.setKSearch(20);        // 设置K近邻数量
    ne.compute(*normals);     // 计算法向量

    // 保存处理结果供可视化线程使用
    *global_cloud = *mls_points;
    *global_normals = *normals;
}

// 主程序入口
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_processing"); // 初始化ROS节点
    ros::NodeHandle nh; // 创建节点句柄
    ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, cloudCallback); // 订阅点云话题，回调处理

    viewer.reset(new pcl::visualization::PCLVisualizer("PCL Viewer")); // 创建PCL可视化对象

    std::thread vis_thread(visualizeCloud); // 启动可视化线程

    ROS_INFO("点云处理与可视化启动！"); // ROS信息提示
    ros::spin(); // 循环等待消息触发回调并执行

    vis_thread.join(); // 等待可视化线程结束
    return 0;
} // main函数结束。程序至此完成点云处理节点的主要执行流程。