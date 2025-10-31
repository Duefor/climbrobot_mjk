#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <cmath>
#include <robot_set/TCPState.h>

using namespace Eigen;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "triangle_traj_pub");
  ros::NodeHandle nh;

  // ros::Publisher pub_pose = nh.advertise<robot_set::TCPState>("/desired_pose_sub", 1);
  ros::Publisher pub_pose = nh.advertise<robot_set::TCPState>("/cartesian_pose", 1);
  ros::Publisher pub_vel  = nh.advertise<robot_set::TCPState>("/desired_vel_sub", 1);

  double side = 0.2;        // 三角形边长 m
  double z = 0.3;           // 平面高度 m
  double period = 15.0;      // 绘制完整三角形时间 s
  double omega = 2*M_PI/period;
  double corner_time = period / 3.0;

  ros::Rate rate(50);
  double t0 = ros::Time::now().toSec();

  Vector3d p0(0.3, 0.0, z);
  Vector3d p1 = p0 + Vector3d(side, 0, 0);
  Vector3d p2 = p0 + Vector3d(side/2, sqrt(3)*side/2, 0);

  while (ros::ok())
  {
    double t = fmod(ros::Time::now().toSec() - t0, period);
    Vector3d pos, vel;

    // 三段线性轨迹
    if (t < corner_time)
    {
      double s = t / corner_time;
      pos = p0 + s*(p1 - p0);
      vel = (p1 - p0) / corner_time;
    }
    else if (t < 2*corner_time)
    {
      double s = (t - corner_time) / corner_time;
      pos = p1 + s*(p2 - p1);
      vel = (p2 - p1) / corner_time;
    }
    else
    {
      double s = (t - 2*corner_time) / corner_time;
      pos = p2 + s*(p0 - p2);
      vel = (p0 - p2) / corner_time;
    }

    // 姿态保持固定
    double rx = 3.14, ry = 0, rz = 3.14;  // 为保证机械臂位置合理

    robot_set::TCPState pose_msg,vel_msg;
    pose_msg.position = {pos.x(), pos.y(), pos.z(), rx, ry, rz};
    vel_msg.velocity  = {vel.x(), vel.y(), vel.z(), 0.0, 0.0, 0.0};

    pub_pose.publish(pose_msg);
    pub_vel.publish(vel_msg);
    // std::cout << "当前期望位姿指令为：" << "[" << pose_msg.position[0] << "," << pose_msg.position[1] << "," << pose_msg.position[2] << "," 
    //     << pose_msg.position[3] << "," << pose_msg.position[4] << "," << pose_msg.position[5] << "]" << std::endl;
    // std::cout << "当前期望速度指令为：" << "[" << vel_msg.velocity[0] << "," << vel_msg.velocity[1] << "," << vel_msg.velocity[2] << "," 
    //     << vel_msg.velocity[3] << "," << vel_msg.velocity[4] << "," << vel_msg.velocity[5] << "]" << std::endl;

    rate.sleep();
  }
  return 0;
}
