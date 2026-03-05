#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

class MyRobotHW : public hardware_interface::RobotHW
{
public:
  MyRobotHW(ros::NodeHandle& nh)
  {
    joint_names_ = {
      "joint_1",
      "joint_2",
      "joint_3",
      "joint_4",
      "joint_5",
      "joint_6"
    };

    int n = joint_names_.size();

    pos_.resize(n);
    vel_.resize(n);
    eff_.resize(n);
    cmd_.resize(n);

    for (int i = 0; i < n; ++i)
    {
      pos_[i] = 0.0;
      vel_[i] = 0.0;
      eff_[i] = 0.0;
      cmd_[i] = 0.0;

      hardware_interface::JointStateHandle state_handle(
          joint_names_[i],
          &pos_[i],
          &vel_[i],
          &eff_[i]);

      joint_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle pos_handle(
          joint_state_interface_.getHandle(joint_names_[i]),
          &cmd_[i]);

      position_joint_interface_.registerHandle(pos_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
  }

  void read()
  {
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      pos_[i] = getJointPositionFromHW(i);
      vel_[i] = getJointVelocityFromHW(i);
      eff_[i] = 0.0;
    }
  }

  void write()
  {
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      sendJointPositionToHW(i, cmd_[i]);
    }
  }

private:

  // ======== 你需要实现的接口 ========

  double getJointPositionFromHW(int index)
  {
    // TODO: 从真实机械臂读取关节角（单位：rad）
    return 0.0;
  }

  double getJointVelocityFromHW(int index)
  {
    // TODO: 从真实机械臂读取关节角速度
    return 0.0;
  }

  void sendJointPositionToHW(int index, double position)
  {
    // TODO: 将目标关节角发送给真实机械臂
  }

  // =================================

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

  std::vector<std::string> joint_names_;
  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> eff_;
  std::vector<double> cmd_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_robot_hw_node");
  ros::NodeHandle nh;

  MyRobotHW robot(nh);

  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Rate rate(100);  // 控制频率 100Hz
  ros::Time last_time = ros::Time::now();

  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    ros::Duration period = now - last_time;
    last_time = now;

    robot.read();
    cm.update(now, period);
    robot.write();

    rate.sleep();
  }

  return 0;
}