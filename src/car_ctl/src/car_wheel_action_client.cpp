#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <car_ctl/CarWheelMotionAction.h>
#include <atomic>
#include <csignal>
#include <cstdlib>

std::atomic<bool> g_cancel_requested(false);

void sigintHandler(int)
{
  g_cancel_requested.store(true);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "car_wheel_action_client", ros::init_options::NoSigintHandler);
  std::signal(SIGINT, sigintHandler);
  std::signal(SIGTERM, sigintHandler);

  if (argc < 4) {
    ROS_INFO("Usage: rosrun car_ctl car_wheel_action_client <mode> <left_value> <right_value> [timeout]");
    ROS_INFO("  mode=0 velocity, mode=1 position");
    ROS_INFO("  left_value/right_value: RPM for velocity mode, revolutions for position mode");
    ROS_INFO("  timeout: optional timeout in seconds (default 10.0)");
    return 1;
  }

  int mode = std::atoi(argv[1]);
  float left_value = std::atof(argv[2]);
  float right_value = std::atof(argv[3]);
  float timeout = 10.0;
  if (argc >= 5) {
    timeout = std::atof(argv[4]);
  }

  actionlib::SimpleActionClient<car_ctl::CarWheelMotionAction> ac("car_wheel_motion", true);
  ROS_INFO("Waiting for car_wheel_motion action server...");
  if (!ac.waitForServer(ros::Duration(10.0))) {
    ROS_ERROR("Action server not available");
    return 1;
  }

  car_ctl::CarWheelMotionGoal goal;
  goal.control_mode = mode;
  goal.left_value = left_value;
  goal.right_value = right_value;
  goal.timeout = timeout;

  ac.sendGoal(goal);
  ROS_INFO("Sent goal: mode=%d left=%.2f right=%.2f timeout=%.2f", mode, left_value, right_value, timeout);

  ros::Rate rate(mode == 0 ? 2 : 10);
  ros::Time start = ros::Time::now();
  double wait_timeout = mode == 0 ? 0.0 : timeout + 5.0;

  while (ros::ok() && !ac.getState().isDone()) {
    if (g_cancel_requested.load()) {
      ROS_WARN("Stop requested. Cancelling car_wheel_motion goal...");
      ac.cancelAllGoals();
      ac.waitForResult(ros::Duration(2.0));
      break;
    }

    if (mode == 0) {
      ROS_INFO_THROTTLE(5, "Velocity goal active...");
    } else if ((ros::Time::now() - start).toSec() > wait_timeout) {
      ROS_WARN("Position goal did not finish within wait time");
      break;
    }

    ros::spinOnce();
    rate.sleep();
  }

  actionlib::SimpleClientGoalState state = ac.getState();
  ROS_INFO("Action finished: %s", state.toString().c_str());
  if (ac.getResult()) {
    ROS_INFO("Result: success=%d message=%s", ac.getResult()->success, ac.getResult()->message.c_str());
  }

  return (state == actionlib::SimpleClientGoalState::SUCCEEDED) ? 0 : 1;
}
