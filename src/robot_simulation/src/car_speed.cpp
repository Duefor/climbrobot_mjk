#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

double x = 0.0;
double y = 0.0;
double yaw = 0.0;

double wheel_radius = 0.15;   // R，两轮半径
double wheel_separation = 0.308; // L，左右轮中心距

double left_wheel_rad = 0.0;
double right_wheel_rad = 0.0;

ros::Time last_time;
ros::Publisher odom_pub;
ros::Publisher joint_state_pub;

void wheelSpeedCb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 2)
        return;

    ros::Time now = ros::Time::now();
    if (last_time.isZero())
    {
        last_time = now;
        return;
    }

    double dt = (now - last_time).toSec();
    last_time = now;
    if(dt > 0.03) return;

    double w_l = msg->data[0]/500.0;
    double w_r = msg->data[1]/500.0;

    double v = wheel_radius * (w_r + w_l) * 0.5;
    double w = wheel_radius * (w_r - w_l) / wheel_separation;

    x += v * cos(yaw) * dt;
    y += v * sin(yaw) * dt;
    yaw += w * dt;

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped tf;

    tf.header.stamp = now;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";

    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    tf.transform.rotation = tf2::toMsg(q);

    br.sendTransform(tf);

    nav_msgs::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = tf.transform.rotation;

    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;

    odom_pub.publish(odom);

    left_wheel_rad -= w_l * dt;
    right_wheel_rad -= w_r * dt;
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = {"joint_left" , "joint_right"};
    joint_state.position = {left_wheel_rad, right_wheel_rad};
    joint_state_pub.publish(joint_state);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_sim_wheel_node");
    ros::NodeHandle nh("~");

    nh.param("wheel_radius", wheel_radius, wheel_radius);
    nh.param("wheel_separation", wheel_separation, wheel_separation);

    ros::Subscriber sub = nh.subscribe("/wheel_speed_cmd", 10, wheelSpeedCb);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    ros::spin();
    return 0;
}