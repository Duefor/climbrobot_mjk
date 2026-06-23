#ifndef CS66_FOLLOW_JOINT_TRAJ_SERVER_H
#define CS66_FOLLOW_JOINT_TRAJ_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <memory>


class CS66FollowTrajectoryServer
{
public:
  // 核心修改：构造函数接收一个 controller 的共享指针
  CS66FollowTrajectoryServer(ros::NodeHandle &nh, std::shared_ptr<CS66RobotController> controller);

  ~CS66FollowTrajectoryServer();

  // 回调函数
  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  
  // 核心修改：这里只存指针，不再初始化
  std::shared_ptr<CS66RobotController> robot_ctrl_;
  
  std::vector<std::string> sdk_joint_names_;

  // 辅助函数声明
  std::vector<int> buildIndexMap(const std::vector<std::string>& in_names);
  void abortGoal(const std::string& msg);
};

#endif