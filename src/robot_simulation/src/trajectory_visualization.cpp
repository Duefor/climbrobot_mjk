// 轨迹可视化
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

double freq = 10;

int main(int argc, char **argv) {
    ros::init(argc, argv, "end_effector_trajectory_publisher");
    ros::NodeHandle nh;

    // 发布轨迹消息
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/ee_trajectory", 10);

    // TF 监听器（用于获取末端位姿）
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    nav_msgs::Path path;
    path.header.frame_id = "base_link";  // 轨迹的参考坐标系（通常为基座）

    ros::Rate rate(freq);  // 10Hz 更新频率

    while (ros::ok()) {
        try {
            // 获取末端坐标系相对于基坐标系的变换
            geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(
                "base_link", "ee_link", ros::Time(0));

            // 将变换转换为位姿
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "base_link";
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.position.z = transform.transform.translation.z;
            pose.pose.orientation = transform.transform.rotation;

            // 将位姿添加到轨迹
            path.poses.push_back(pose);
            // 保留最近 100 个位姿
            if (path.poses.size() > 1000) {
                path.poses.erase(path.poses.begin());
            }
            path.header.stamp = ros::Time::now();
            path_pub.publish(path);

        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

        rate.sleep();
    }
    return 0;
}