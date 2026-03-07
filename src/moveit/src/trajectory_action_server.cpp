#include <ros/ros.h>
#include <ros/param.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

#include <robot_sdk_wrapper/robot_sdk.h>

std::string DEFAULT_ROBOT_IP;   // 机械臂ip
std::string DEFAULT_PC_IP;  // PCip
bool MODE;
std::string external_control_file_address;  // 外部控制文件
std::string output_recipe_file_address; // 外部控制文件
std::string input_recipe_file_address;  // 外部控制文件
std::string task_file_address;    // 机械臂配置/任务文件名
std::string jointNames_topic;   // 关节参数服务器名称

std::string joint_state_pubTopic;   // 关节角话题
double control_freq;    // 机械臂轨迹跟随控制频率
int IA;     // 机械臂轨迹插值算法

class ArmTrajectoryActionServer
{
public:
  ArmTrajectoryActionServer(ros::NodeHandle& nh, EliteCSRobotSDK& robot) : nh_(nh), robot_(robot), action_server_(nh_, "planning_group_controller/follow_joint_trajectory", boost::bind(&ArmTrajectoryActionServer::executeCB, this, _1), false)
  {
    // 读取关节名称参数
    ros::Time start = ros::Time::now();
    while (ros::ok())
    {
        if (nh_.getParam(jointNames_topic, joint_names_)) break;
        if (ros::Time::now() - start > ros::Duration(2.0))
        {
            ROS_ERROR("Timeout waiting for param robot joint names");
            ros::shutdown();
            return;
        }
        ros::Duration(0.05).sleep();  // 每50ms检查一次
    }

    action_server_.start();
    ROS_INFO("FollowJointTrajectory Action Server Started");

    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_state_pubTopic, 10);
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

  std::atomic<bool> executing_{false};
  control_msgs::FollowJointTrajectoryGoalConstPtr active_goal_;

  bool readJointState(std::vector<double>& positions)
  {
    auto current_joint = robot_.getCurrentJoint();
    positions.resize(joint_names_.size(), 0.0);
    if(current_joint.size() != positions.size()) return false;
    for(size_t i = 0; i < positions.size(); ++i)
    {
        positions[i] = current_joint[i];
    }
    return true;
  }


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
        for(size_t j = 0; j < goal->trajectory.joint_names.size(); ++j)
        {
            p.positions[j] = goal->trajectory.points[i].positions[j];
            p.velocities[j] = goal->trajectory.points[i].velocities[j];
            p.accelerations[j] = goal->trajectory.points[i].accelerations[j];
            p.time_from_start = goal->trajectory.points[i].time_from_start.toSec();
        }
        traj.push_back(p);
    }

    executing_ = true;
    active_goal_ = goal;
    bool ok = robot_.ExecuteJointTrajectory(traj, control_freq, IA);
    executing_ = false;
    active_goal_.reset();

    if(!ok)
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
        if(executing_ && active_goal_)
        {
            publishFeedback(active_goal_);
        }

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

    ros::param::param(std::string("~DEFAULT_ROBOT_IP"), DEFAULT_ROBOT_IP, std::string("192.168.1.199"));
    ros::param::param(std::string("~DEFAULT_PC_IP"), DEFAULT_PC_IP, std::string("192.168.1.150"));
    ros::param::param(std::string("~MODE"), MODE, true);
    ros::param::param(std::string("~external_control_file_address"), external_control_file_address, std::string("external_control.script"));
    ros::param::param(std::string("~output_recipe_file_address"), output_recipe_file_address, std::string("output_recipe.txt"));
    ros::param::param(std::string("~input_recipe_file_address"), input_recipe_file_address, std::string("input_recipe.txt"));
    ros::param::param(std::string("~task_file_address"), task_file_address, std::string("mjktest.task"));
    ros::param::param(std::string("~jointNames_topic"), jointNames_topic, std::string("/joint_names"));

    ros::param::param(std::string("~joint_state_pubTopic"), joint_state_pubTopic, std::string("/joint_states"));
    ros::param::param(std::string("~control_freq"), control_freq, double(200.0));
    ros::param::param(std::string("~IA"), IA, int(1));


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