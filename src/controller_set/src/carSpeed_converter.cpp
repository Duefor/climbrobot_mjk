#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <algorithm>

// 最大速度
constexpr double MAX_FRONT_SPEED = 1000.0;
constexpr double MAX_BACK_SPEED  = 150.0;

constexpr double MAX_TURN_SPEED  = 1000.0;

// 转向（joint0）
constexpr double STEER_DEADZONE = 15.0;
constexpr double STEER_MAX      = 60.0;

// 前进 / 后退（joint1）
constexpr double FORWARD_MIN = 30.0;
constexpr double FORWARD_MAX = 105.0;

constexpr double BACKWARD_MIN = 0.0;
constexpr double BACKWARD_MAX = 14.0;

ros::Subscriber touch_joint_sub;
ros::Publisher  car_speed_pub;

void cb(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (msg->position.size() < 2)
    {
        ROS_WARN_THROTTLE(1.0, "JointState size < 2");
        return;
    }

    const double joint0 = msg->position[0] * 180.0 / M_PI;
    const double joint1 = msg->position[1] * 180.0 / M_PI;

    double v = 0.0;   // 线速度
    double w = 0.0;

    if (std::fabs(joint0) > STEER_DEADZONE)
    {
        double scale = (std::fabs(joint0) - STEER_DEADZONE) / (STEER_MAX - STEER_DEADZONE);
        scale = std::min(scale, 1.0);

        w = scale * MAX_TURN_SPEED;

        if (joint0 < 0.0)
            w = -w;
    }

    if (joint1 > FORWARD_MIN)
    {
        double scale = (joint1 - FORWARD_MIN) / (FORWARD_MAX - FORWARD_MIN);
        scale = std::min(scale, 1.0);

        v = scale * MAX_FRONT_SPEED;
    }
    else if (joint1 < BACKWARD_MAX)
    {
        double scale = (BACKWARD_MAX - joint1) / (BACKWARD_MAX - BACKWARD_MIN);
        scale = std::min(scale, 1.0);

        v = -scale * MAX_BACK_SPEED;
    }

    // // 角速度与线速度耦合(无法单独旋转)
    // double speed_ratio = std::min(std::fabs(v) / MAX_FRONT_SPEED, 1.0);
    // w = w * speed_ratio;

    w = w * 0.5;

    std_msgs::Float64MultiArray car_vel;
    car_vel.data.resize(2);

    car_vel.data[0] = v - w;   // left
    car_vel.data[1] = v + w;   // right

    car_speed_pub.publish(car_vel);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "carSpeed_converter");
    ros::NodeHandle nh;

    car_speed_pub  = nh.advertise<std_msgs::Float64MultiArray>("/wheel_speed_cmd", 10);
    touch_joint_sub = nh.subscribe("/phantom/joint_states", 10, cb);

    ROS_INFO("carSpeed_converter running");
    ros::spin();
    return 0;
}