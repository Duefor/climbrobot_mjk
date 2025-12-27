// transform_publisher_simple.cpp
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>
#include <robot_set/TCPState.h>

// 旋转向量转四元数
void rotationVectorToQuaternion(double rx, double ry, double rz, 
                               double& qx, double& qy, double& qz, double& qw) {
    double angle = sqrt(rx*rx + ry*ry + rz*rz);
    
    if (angle < 1e-10) {
        qx = 0.0;
        qy = 0.0;
        qz = 0.0;
        qw = 1.0;
        return;
    }
    
    double half_angle = angle * 0.5;
    double sin_half = sin(half_angle);
    
    qx = (rx / angle) * sin_half;
    qy = (ry / angle) * sin_half;
    qz = (rz / angle) * sin_half;
    qw = cos(half_angle);
}

// 位姿回调函数
void poseCallback(const robot_set::TCPState::ConstPtr& msg, 
                  tf2_ros::TransformBroadcaster* tf_broadcaster) {
    if (msg->position.size() < 6) {
        ROS_WARN("Invalid pose message: expected 6 values, got %lu", msg->position.size());
        return;
    }
    
    // 解析消息
    float x = msg->position[0];
    float y = msg->position[1];
    float z = msg->position[2];
    float rx = msg->position[3];
    float ry = msg->position[4];
    float rz = msg->position[5];
    
    // 创建tf消息
    geometry_msgs::TransformStamped transformStamped;
    
    // 设置header
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "static_frame";  // 父坐标系
    transformStamped.child_frame_id = "dynamic_frame";  // 子坐标系
    
    // 设置平移
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;
    
    // 旋转向量转四元数
    double qx, qy, qz, qw;
    rotationVectorToQuaternion(rx, ry, rz, qx, qy, qz, qw);
    
    // 设置旋转
    transformStamped.transform.rotation.x = qx;
    transformStamped.transform.rotation.y = qy;
    transformStamped.transform.rotation.z = qz;
    transformStamped.transform.rotation.w = qw;
    
    // 发布tf
    tf_broadcaster->sendTransform(transformStamped);
    
    ROS_DEBUG("Published transform: [%f, %f, %f], [%f, %f, %f]", 
              x, y, z, rx, ry, rz);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_frame_publisher_simple");
    ros::NodeHandle nh;
    
    // 创建TF广播器
    tf2_ros::TransformBroadcaster tf_broadcaster;
    
    // 订阅位姿话题
    ros::Subscriber pose_sub = nh.subscribe<robot_set::TCPState>(
        "/phantom/pose", 10, 
        boost::bind(poseCallback, _1, &tf_broadcaster)
    );
    
    ROS_INFO("Transform publisher started, listening to /dynamic_pose");
    
    ros::spin();
    
    return 0;
}