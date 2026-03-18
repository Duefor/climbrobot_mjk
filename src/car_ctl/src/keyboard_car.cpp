#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <map>

// 非阻塞读取
int getKey()
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
    return ch;
  return -1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_keyboard_control");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/phantom/joint_states_car", 10);

  double fixed_angle = 45.0/180.0*M_PI; // 30°

  // 状态变量（toggle）
  bool j1_pos = false, j1_neg = false;
  bool j2_pos = false, j2_neg = false;
  bool j4_pos = false, j4_neg = false;

  // 防止长按重复触发
  std::map<int, bool> key_pressed;

  ros::Rate rate(50);

  while (ros::ok())
  {
    int key = getKey();

    if (key != -1)
    {
      // 如果之前没按过 → 触发一次
      if (!key_pressed[key])
      {
        switch (key)
        {
        case 65: // ↑ → joint2 +
          j2_pos = !j2_pos;
          j2_neg = false;
          break;
        case 66: // ↓ → joint2 -
          j2_neg = !j2_neg;
          j2_pos = false;
          break;
        case 67: // → → joint1 +
          j1_pos = !j1_pos;
          j1_neg = false;
          break;
        case 68: // ← → joint1 -
          j1_neg = !j1_neg;
          j1_pos = false;
          break;
        case 'q': // joint4 +
          j4_pos = !j4_pos;
          j4_neg = false;
          break;
        case 'e': // joint4 -
          j4_neg = !j4_neg;
          j4_pos = false;
          break;
        default:
          break;
        }
        key_pressed[key] = true;
      }
    }
    else
    {
      // 没有输入 → 清空按键状态（允许下一次触发）
      key_pressed.clear();
    }

    // 根据状态输出角度
    double joint1 = (j1_pos ? fixed_angle : 0.0) + (j1_neg ? -fixed_angle : 0.0);
    double joint2 = (j2_pos ? fixed_angle : 0.0) + (j2_neg ? -fixed_angle : 0.0);
    double joint4 = (j4_pos ? fixed_angle : 0.0) + (j4_neg ? -fixed_angle : 0.0);
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.name = {"joint1", "joint2","joint3", "joint4","joint5","joint6"};
    msg.position = {joint1, joint2+15.0/180.0*M_PI ,0.0, joint4+3.1414822448183592 ,0.0,0.0};

    // double joint1 = (j1_pos ? fixed_angle : 0.0) + (j1_neg ? -fixed_angle : 0.0);
    // double joint2 = (j2_pos ? fixed_angle : 0.0) + (j2_neg ? -fixed_angle : 0.0);
    // sensor_msgs::JointState msg;
    // msg.header.stamp = ros::Time::now();
    // msg.name = {"joint1", "joint2","joint3", "joint4","joint5","joint6"};
    // msg.position = {joint1, joint2+15.0/180.0*M_PI ,0.0, 0.0 ,0.0,0.0};


    pub.publish(msg);

    rate.sleep();
  }

  return 0;
}