#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <car_ctl/CarAutoTaskAction.h>
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
  ros::init(argc, argv, "car_auto_task_client", ros::init_options::NoSigintHandler);
  std::signal(SIGINT, sigintHandler);
  std::signal(SIGTERM, sigintHandler);

  if (argc != 2) {
    ROS_ERROR("Usage: rosrun car_ctl car_auto_task_client <distance_m>");
    return 1;
  }

  double distance = std::atof(argv[1]);

  actionlib::SimpleActionClient<car_ctl::CarAutoTaskAction> client("car_auto_task", true);

  ROS_INFO("Waiting for car_auto_task action server...");
  if (!client.waitForServer(ros::Duration(20.0))) {
    ROS_ERROR("car_auto_task action server not available after waiting");
    return 1;
  }

  car_ctl::CarAutoTaskGoal goal;
  goal.distance = static_cast<float>(distance);

  ROS_INFO("Sending goal: distance=%.3f m", distance);
  client.sendGoal(goal);

  ros::Rate rate(10);
  while (ros::ok() && !client.getState().isDone()) {
    if (g_cancel_requested.load()) {
      ROS_WARN("Stop requested. Cancelling car_auto_task goal...");
      client.cancelAllGoals();
      client.waitForResult(ros::Duration(2.0));
      break;
    }
    rate.sleep();
  }

  actionlib::SimpleClientGoalState state = client.getState();
  ROS_INFO("Finished: %s", state.toString().c_str());

  if (client.getResult()) {
    const car_ctl::CarAutoTaskResultConstPtr& result = client.getResult();
    ROS_INFO("Result: success=%d message=%s", result->success, result->message.c_str());
  } else {
    ROS_WARN("No result received");
  }

  return (state == actionlib::SimpleClientGoalState::SUCCEEDED) ? 0 : 1;
}
