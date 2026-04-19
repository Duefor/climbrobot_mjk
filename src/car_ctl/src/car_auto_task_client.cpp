#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <car_ctl/CarAutoTaskAction.h>
#include <signal.h>

// 全局指针，供信号回调使用
actionlib::SimpleActionClient<car_ctl::CarAutoTaskAction>* g_client_ptr = nullptr;

// Ctrl+C 信号回调
void sigintHandler(int sig)
{
  if (g_client_ptr) {
    ROS_WARN("Ctrl+C detected! Cancelling goal...");
    g_client_ptr->cancelAllGoals();  // 或 cancelGoal()
  }

  ros::shutdown();  // 正常关闭 ROS
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "car_auto_task_client", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigintHandler);  // 注册 Ctrl+C 处理

  if (argc != 2) {
    ROS_ERROR("Usage: rosrun car_ctl car_auto_task_client <distance_m>");
    return 1;
  }

  double distance = std::atof(argv[1]);

  actionlib::SimpleActionClient<car_ctl::CarAutoTaskAction> client("car_auto_task", true);
  g_client_ptr = &client;  // 赋值给全局变量

  ROS_INFO("Waiting for car_auto_task action server...");
  if (!client.waitForServer(ros::Duration(20.0))) {
    ROS_ERROR("car_auto_task action server not available after waiting");
    return 1;
  }

  car_ctl::CarAutoTaskGoal goal;
  goal.distance = static_cast<float>(distance);

  ROS_INFO("Sending goal: distance=%.3f m", distance);
  client.sendGoal(goal);

  // 循环等待，而不是直接 waitForResult
  ros::Rate rate(10);
  while (ros::ok() && !client.getState().isDone()) {
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