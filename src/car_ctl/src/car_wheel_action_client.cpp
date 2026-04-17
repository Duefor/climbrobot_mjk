#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <car_ctl/CarWheelMotionAction.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "car_wheel_action_client");
  if (argc < 4) {
    ROS_INFO("Usage: rosrun car_ctl car_wheel_action_client <mode> <left_value> <right_value> [timeout]");
    ROS_INFO("  mode=0 velocity, mode=1 position");
    ROS_INFO("  left_value/right_value: RPM for velocity mode, revolutions for position mode");
    ROS_INFO("  timeout: optional position mode timeout in seconds (default 10.0)");
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

  if (mode == 0) {
    ROS_INFO("Velocity mode: goal will remain active until canceled by Ctrl-C or a new goal.");
    ros::Rate rate(2);
    while (ros::ok() && !ac.getState().isDone()) {
      ROS_INFO_THROTTLE(5, "Velocity goal active...");
      ros::spinOnce();
      rate.sleep();
    }
  } else {
    bool finished = ac.waitForResult(ros::Duration(timeout + 5.0));
    if (!finished) {
      ROS_WARN("Position goal did not finish within wait time");
    }
  }

  actionlib::SimpleClientGoalState state = ac.getState();
  ROS_INFO("Action finished: %s", state.toString().c_str());
  if (ac.getResult()) {
    ROS_INFO("Result: success=%d message=%s", ac.getResult()->success, ac.getResult()->message.c_str());
  }

  return 0;
}
