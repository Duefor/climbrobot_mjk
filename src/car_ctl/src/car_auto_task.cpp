#include <cmath>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <car_ctl/CarAutoTaskAction.h>
#include <car_ctl/CarWheelMotionAction.h>

class CarAutoTaskActionServer
{
public:
  typedef actionlib::SimpleActionClient<car_ctl::CarWheelMotionAction> WheelActionClient;

  CarAutoTaskActionServer(const std::string& name)
    : nh_(ros::NodeHandle()),
      as_(nh_, name, boost::bind(&CarAutoTaskActionServer::executeCB, this, _1), false),
      ac_("car_wheel_motion", true),
      action_name_(name),
      wheel_radius_(0.15),
      wheel_separation_(0.308),
      base_turn_timeout_(15.0),
      base_drive_timeout_(30.0)
  {
    ros::NodeHandle pnh("~");
    pnh.param("wheel_radius", wheel_radius_, wheel_radius_);
    pnh.param("wheel_separation", wheel_separation_, wheel_separation_);
    pnh.param("base_turn_timeout", base_turn_timeout_, base_turn_timeout_);
    pnh.param("base_drive_timeout", base_drive_timeout_, base_drive_timeout_);

    ROS_INFO("[%s] Waiting for car_wheel_motion action server...", action_name_.c_str());
    if (!ac_.waitForServer(ros::Duration(20.0))) {
      ROS_FATAL("[%s] car_wheel_motion action server not available", action_name_.c_str());
      ros::shutdown();
      return;
    }

    as_.start();
    ROS_INFO("[%s] car_auto_task action server started", action_name_.c_str());
  }

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<car_ctl::CarAutoTaskAction> as_;
  WheelActionClient ac_;
  std::string action_name_;
  double wheel_radius_;
  double wheel_separation_;
  double base_turn_timeout_;
  double base_drive_timeout_;

  static double normalizeAngle(double value)
  {
    while (value > M_PI) value -= 2.0 * M_PI;
    while (value < -M_PI) value += 2.0 * M_PI;
    return value;
  }

  // 发送底盘转动时每轮的转数，计算公式基于差速转动时轮子绕车体中心运动的弧长与轮子周长的关系
  double computeRotationRevolutions() const
  {
    // 两轮差速转动时，每个轮子绕车体中心运动的弧长为 (L/2) * theta
    // 90° 转向对应 theta = pi/2，因此每轮圈数为 (L/2 * theta) / (2*pi*R) = L / (8*R)
    return wheel_separation_ / (8.0 * wheel_radius_);
  }

  // 发送底盘前进时每轮的转数，计算公式基于前进距离与轮子周长的关系
  double computeDistanceRevolutions(double distance) const
  {
    // 前进距离与轮子圈数的线性关系：distance = 2*pi*R*revolutions
    return distance / (2.0 * M_PI * wheel_radius_);
  }

  bool sendWheelPositionGoal(double left_revs, double right_revs, double timeout, const ros::Time& deadline)
  {
    car_ctl::CarWheelMotionGoal wheel_goal;
    wheel_goal.control_mode = 0;
    wheel_goal.left_value = static_cast<float>(left_revs);
    wheel_goal.right_value = static_cast<float>(right_revs);
    wheel_goal.timeout = static_cast<float>(timeout);

    ROS_INFO("[%s] Sending wheel position goal: left=%.4f rev, right=%.4f rev, timeout=%.1f", action_name_.c_str(), left_revs, right_revs, timeout);
    ac_.sendGoal(wheel_goal);

    ros::Rate rate(20);
    while (ros::ok() && ros::Time::now() < deadline) {
      if (as_.isPreemptRequested()) {
        ROS_WARN("[%s] Preempt requested, canceling wheel position goal", action_name_.c_str());
        ac_.cancelGoal();
        return false;
      }
      if (ac_.getState().isDone()) {
        break;
      }
      ros::spinOnce();
      rate.sleep();
    }

    if (!ac_.getState().isDone()) {
      ROS_ERROR("[%s] Wheel position goal timed out", action_name_.c_str());
      ac_.cancelGoal();
      return false;
    }

    if (ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
      auto result = ac_.getResult();
      std::string result_msg = result ? result->message : "No result available";
      ROS_ERROR("[%s] Wheel action failed: %s (Result: %s)", action_name_.c_str(), ac_.getState().toString().c_str(), result_msg.c_str());
      return false;
    }

    ROS_INFO("[%s] Wheel position goal succeeded", action_name_.c_str());
    return true;
  }

  void executeCB(const car_ctl::CarAutoTaskGoalConstPtr& goal)
  {
    car_ctl::CarAutoTaskResult result;
    car_ctl::CarAutoTaskFeedback feedback;

    ROS_INFO("[%s] Received goal: distance=%.3f m", action_name_.c_str(), goal->distance);

    double rotation_revs = computeRotationRevolutions();
    double distance_revs = computeDistanceRevolutions(goal->distance);
    double turn_timeout = std::max(base_turn_timeout_, std::fabs(rotation_revs) * 10.0);
    double drive_timeout = std::max(base_drive_timeout_, std::fabs(distance_revs) * 10.0);
    ros::Time overall_deadline = ros::Time::now() + ros::Duration(turn_timeout + drive_timeout + 10.0);

    feedback.progress = 0.0;
    as_.publishFeedback(feedback);

    // 1. Clockwise 90° turn
    if (!sendWheelPositionGoal(rotation_revs, -rotation_revs, turn_timeout, overall_deadline)) {
      result.success = false;
      result.message = "Failed during first 90-degree rotation";
      as_.setAborted(result);
      return;
    }
    feedback.progress = 0.33f;
    as_.publishFeedback(feedback);

    // 2. Forward move
    if (!sendWheelPositionGoal(distance_revs, distance_revs, drive_timeout, overall_deadline)) {
      result.success = false;
      result.message = "Failed during forward distance move";
      as_.setAborted(result);
      return;
    }
    feedback.progress = 0.66f;
    as_.publishFeedback(feedback);

    // 3. Counter-clockwise 90° turn
    if (!sendWheelPositionGoal(-rotation_revs, rotation_revs, turn_timeout, overall_deadline)) {
      result.success = false;
      result.message = "Failed during second 90-degree rotation";
      as_.setAborted(result);
      return;
    }

    feedback.progress = 1.0f;
    as_.publishFeedback(feedback);
    result.success = true;
    result.message = "Completed rotate-forward-rotate maneuver using wheel position control";
    as_.setSucceeded(result);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "car_auto_task_server");
  CarAutoTaskActionServer server("car_auto_task");
  ros::spin();
  return 0;
}
