#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

int getKey()
{
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_keyboard_control");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

  double joint1 = 0.0; // 第一轴
  double joint2 = 0.0; // 第二轴
  double step = 0.05;  // 每次按键变化弧度约3°

  ROS_INFO("Use arrow keys to control joints. Press 'q' to quit.");

  ros::Rate rate(20);
  while (ros::ok())
  {
    int key = getKey();

    if (key == 'q')
      break;

    switch (key)
    {
    case 65: // ↑
      joint2 += step;
      break;
    case 66: // ↓
      joint2 -= step;
      break;
    case 67: // →
      joint1 += step;
      break;
    case 68: // ←
      joint1 -= step;
      break;
    default:
      break;
    }

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.name = {"joint1", "joint2"};
    msg.position = {joint1, joint2};

    pub.publish(msg);

    // std::cout << "\rJoint1: " << joint1 << " rad, Joint2: " << joint2 << " rad" << std::flush;
    // std::cout << std::endl;

    rate.sleep();
  }

  std::cout << std::endl;
  return 0;
}
