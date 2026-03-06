#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

#include <robot_sdk_wrapper/robot_sdk.h>

std::string DEFAULT_ROBOT_IP = "192.168.1.199";   // 机械臂ip
std::string DEFAULT_PC_IP = "192.168.1.150";  // PCip
bool MODE = true;
std::string external_control_file_address = "/home/barry/workspace/climbrobot_mjk/src/robot_sdk_wrapper/resource/external_control.script";  // 外部控制文件
std::string output_recipe_file_address = "/home/barry/workspace/climbrobot_mjk/src/robot_sdk_wrapper/resource/output_recipe.txt"; // 外部控制文件
std::string input_recipe_file_address = "/home/barry/workspace/climbrobot_mjk/src/robot_sdk_wrapper/resource/input_recipe.txt";  // 外部控制文件
std::string task_file_address = "mjktest.task";    // 机械臂配置/任务文件名

class ArmTrajectoryActionServer
{
public:
  ArmTrajectoryActionServer(ros::NodeHandle& nh, EliteCSRobotSDK& robot) : nh_(nh), robot_(robot), action_server_(nh_, "planning_group_controller/follow_joint_trajectory", boost::bind(&ArmTrajectoryActionServer::executeCB, this, _1), false)
  {
    // 读取关节名称参数
    if (!nh_.getParam("/move_group/planning_group_controller/joints", joint_names_))
    {
      ROS_ERROR("No joint names specified in parameter 'joints'");
      ros::shutdown();
    }

    action_server_.start();
    ROS_INFO("FollowJointTrajectory Action Server Started");

    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    running_ = true;
    joint_state_thread_ = std::thread(&ArmTrajectoryActionServer::jointStateLoop, this);
  }

  ~ArmTrajectoryActionServer()
  {
    running_ = false;
    if (joint_state_thread_.joinable()) joint_state_thread_.join();
  }

private:
  ros::NodeHandle nh_;
  EliteCSRobotSDK& robot_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> action_server_;

  std::vector<std::string> joint_names_;

  control_msgs::FollowJointTrajectoryFeedback feedback_;
  control_msgs::FollowJointTrajectoryResult result_;

  ros::Publisher joint_state_pub_;
  std::thread joint_state_thread_;
  std::atomic<bool> running_;

  /* =========================
     你需要实现的硬件接口函数
     ========================= */

  bool readJointState(std::vector<double>& positions)
  {
    // TODO: 从真实机械臂读取当前关节角
    // 填充 positions
    auto cuurentjoint = robot_.getCurrentJoint();
    positions.resize(joint_names_.size(), 0.0);
    if(cuurentjoint.size() != positions.size()) return false;
    for(int i = 0; i < positions.size(); i++)
    {
        positions[i] = cuurentjoint[i];
    }
    return true;
  }

  /* ========================= */

  bool validateGoal(
      const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
  {
    if (goal->trajectory.joint_names != joint_names_)
    {
      ROS_ERROR("Joint names mismatch");
      return false;
    }

    if (goal->trajectory.points.empty())
    {
      ROS_ERROR("Trajectory is empty");
      return false;
    }

    return true;
  }

  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
  {
    ROS_INFO("Received new trajectory goal");

    if (!validateGoal(goal))
    {
      result_.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL; action_server_.setAborted(result_);
      return;
    }

    // 构造sdk中适配的轨迹点
    std::vector<EliteCSRobotSDK::TrajectoryPoint> traj;
    for(size_t i = 0; i < goal->trajectory.points.size(); ++i)
    {
        EliteCSRobotSDK::TrajectoryPoint p;
        for(int j = 0; j < goal->trajectory.joint_names.size(); ++j)
        {
            p.positions[j] = goal->trajectory.points[i].positions[j];
            p.velocities[j] = goal->trajectory.points[i].velocities[j];
            p.accelerations[j] = goal->trajectory.points[i].accelerations[j];
            p.time_from_start = goal->trajectory.points[i].time_from_start.toSec();
        }
        traj.push_back(p);
    }

    if(!robot_.ExecuteJointTrajectory(traj))
    {
        ROS_ERROR("Hardware command failed");
        result_.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
        action_server_.setAborted(result_);
        return;
    }

    ROS_INFO("Trajectory execution completed");
    result_.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    action_server_.setSucceeded(result_);
  }

  void publishFeedback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
  {
    std::vector<double> current_positions;

    if (!readJointState(current_positions))
      return;

    feedback_.joint_names = joint_names_;
    feedback_.actual.positions = current_positions;
    feedback_.desired = goal->trajectory.points.back();
    feedback_.header.stamp = ros::Time::now();

    action_server_.publishFeedback(feedback_);
  }

  void jointStateLoop()
  {
    ros::Rate rate(100);

    while (ros::ok() && running_)
    {
        std::vector<double> positions;

        if (readJointState(positions))
        {
            sensor_msgs::JointState msg;
            msg.header.stamp = ros::Time::now();
            msg.name = joint_names_;
            msg.position = positions;

            joint_state_pub_.publish(msg);
        }

        rate.sleep();
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_action_server");
  ros::NodeHandle nh;

  EliteCSRobotSDK cs66robot(
        DEFAULT_ROBOT_IP,
        DEFAULT_PC_IP,
        MODE,
        external_control_file_address,
        output_recipe_file_address,
        input_recipe_file_address,
        task_file_address,
        250
    );

    if (!cs66robot.init() || !cs66robot.start())
    {
        std::cerr << "Robot init/start failed" << std::endl;
        return 1;
    }

  ArmTrajectoryActionServer server(nh, cs66robot);

  ros::spin();
  return 0;
}