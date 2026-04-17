#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <car_ctl/CarAutoTaskAction.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "car_auto_task_client");
  if (argc != 2) {
    ROS_ERROR("Usage: rosrun car_ctl car_auto_task_client <distance_m>");
    return 1;
  }

  double distance = std::atof(argv[1]);
  if (!ros::ok()) {
    return 1;
  }

  actionlib::SimpleActionClient<car_ctl::CarAutoTaskAction> client("car_auto_task", true);
  ROS_INFO("Waiting for car_auto_task action server...");
  if (!client.waitForServer(ros::Duration(20.0))) {
    ROS_ERROR("car_auto_task action server not available after waiting");
    return 1;
  }

  car_ctl::CarAutoTaskGoal goal;
  goal.distance = static_cast<float>(distance);

  ROS_INFO("Sending car_auto_task goal: distance=%.3f m", distance);
  client.sendGoal(goal);

  bool finished_before_timeout = client.waitForResult(ros::Duration(60.0));
  if (!finished_before_timeout) {
    ROS_WARN("car_auto_task did not finish before timeout");
  }

  actionlib::SimpleClientGoalState state = client.getState();
  ROS_INFO("car_auto_task finished: %s", state.toString().c_str());

  if (client.getResult()) {
    const car_ctl::CarAutoTaskResultConstPtr& result = client.getResult();
    ROS_INFO("Result: success=%d message=%s", result->success, result->message.c_str());
  } else {
    ROS_WARN("No result received from car_auto_task action server");
  }

  return (state == actionlib::SimpleClientGoalState::SUCCEEDED) ? 0 : 1;
}
