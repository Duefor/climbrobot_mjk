#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>

ros::Publisher pub_wheel;

void jointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  if (msg->position.size() < 2)
    return;

  double joint1 = msg->position[0];
  double joint2 = msg->position[1];

  double j1_deg = joint1 * 180.0 / M_PI;    // 弧度转角度
  double j2_deg = joint2 * 180.0 / M_PI;

  double dead_zone = 15.0;  // 界限，该度数这下不执行命令
  double max_angle = 90.0;  // 最大角度
  double max_speed = 500.0;   // 单轮最大速度(rpm)，实际最大速度是这个的两倍

  double turn_speed = 0.0;
  double move_speed = 0.0;

  // 转向控制（第一轴）
  if (fabs(j1_deg) > dead_zone)
  {
    double ratio = std::min((fabs(j1_deg) - dead_zone) / (max_angle - dead_zone), 1.0);
    turn_speed = max_speed * ratio * ((j1_deg > 0) ? 1.0 : -1.0);
  }

  // 前后控制（第二轴）
  if (fabs(j2_deg) > dead_zone)
  {
    double ratio = std::min((fabs(j2_deg) - dead_zone) / (max_angle - dead_zone), 1.0);
    move_speed = max_speed * ratio * ((j2_deg > 0) ? 1.0 : -1.0);
  }

  double left_vel = move_speed - turn_speed;
  double right_vel = move_speed + turn_speed;

  std_msgs::Float64MultiArray wheel_msg;
  wheel_msg.data = {left_vel, right_vel};
  pub_wheel.publish(wheel_msg);
  std::cout << "当前速度为[左轮，右轮]：" << "[" << wheel_msg.data[0] << "," << wheel_msg.data[1] << "]" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle nh;

  pub_wheel = nh.advertise<std_msgs::Float64MultiArray>("/wheel_speed_cmd", 1);
  ros::Subscriber sub_joint = nh.subscribe("/joint_states", 1, jointCallback);

  ros::spin();

  return 0;
}
