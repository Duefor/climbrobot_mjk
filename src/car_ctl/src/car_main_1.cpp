#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <car_ctl/CarWheelMotionAction.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <string>
#include <std_msgs/Float64MultiArray.h>

class CarWheelMotionActionServer
{
public:
  CarWheelMotionActionServer(const std::string& name)
    : nh_(ros::NodeHandle()),
      as_(nh_, name, boost::bind(&CarWheelMotionActionServer::executeCB, this, _1), false),
      action_name_(name),
      sock_(-1),
      sim_mode_(false),
      current_mode_(-1),
      speed_mode_initialized_(false),
      position_mode_initialized_(false),
      stop_requested_(false),
      emergency_stop_sent_(false)
  {
    ros::NodeHandle pnh("~");
    pnh.param("can_interface", can_interface_, std::string("can0"));    // CAN接口名称
    pnh.param("pulse_per_rev", pulse_per_rev_, 10000);  // 每转脉冲数(用于位置控制)
    pnh.param("gear_ratio", gear_ratio_, 500);  // 减速比(用于位置控制)
    pnh.param("position_timeout", position_timeout_, 10.0); // 位置控制超时时间(秒)
    pnh.param("speed_publish_hz", speed_publish_hz_, 200);  // 速度控制时发布速度命令的频率(Hz)
    pnh.param("sim_mode", sim_mode_, false);

    wheel_speed_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/wheel_speed_cmd", 10);

    if (!sim_mode_) {
      sock_ = setupCanSocket(can_interface_, addr_);
      if (sock_ < 0) {
        ROS_FATAL("Failed to open CAN socket on %s", can_interface_.c_str());
        ros::shutdown();
        return;
      }
    }

    as_.start();
    ROS_INFO("[%s] Car wheel motion action server started on %s", action_name_.c_str(), sim_mode_ ? "SIM" : can_interface_.c_str());
  }

  ~CarWheelMotionActionServer()
  {
    emergencyStop();
    if (sock_ >= 0) {
      close(sock_);
      sock_ = -1;
    }
  }

  void emergencyStop()
  {
    stop_requested_.store(true);

    bool already_sent = emergency_stop_sent_.exchange(true);
    if (already_sent) {
      return;
    }

    if (sim_mode_) {
      publishWheelSpeedCommand(0, 0);
      return;
    }

    if (sock_ < 0) {
      return;
    }

    ROS_WARN("[%s] Emergency stop: sending zero speed and stop commands", action_name_.c_str());
    sendZeroSpeed();
    sendstop();
  }

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<car_ctl::CarWheelMotionAction> as_;
  std::string action_name_;
  std::string can_interface_;
  int sock_;
  struct sockaddr_can addr_;
  int pulse_per_rev_;
  int gear_ratio_;
  double position_timeout_;
  int speed_publish_hz_;
  bool sim_mode_;
  int current_mode_;
  bool speed_mode_initialized_;
  bool position_mode_initialized_;
  std::atomic<bool> stop_requested_;
  std::atomic<bool> emergency_stop_sent_;
  ros::Publisher wheel_speed_pub_;

  // 将RPM转换为CAN协议中的速度值
  static int32_t rpmToVelocity(int rpm)
  {
    return rpm * 10000 / 60;    // 转换为每分钟脉冲数,rpm是每分钟转数，乘以每转脉冲数除以60秒
  }

  // 初始化CAN通信接口
  int setupCanSocket(const std::string& ifname, struct sockaddr_can& addr)
  {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
      perror("Socket creation error");
      return -1;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
      perror("SIOCGIFINDEX error");
      close(s);
      return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      perror("Bind error");
      close(s);
      return -1;
    }

    return s;
  }

  // 发送CAN帧
  void sendCanFrame(int s, const struct sockaddr_can& addr, int can_id, const unsigned char data[8], int len = 8)
  {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.can_dlc = len;
    std::memcpy(frame.data, data, len);
    int nbytes = sendto(s, &frame, sizeof(frame), 0, (struct sockaddr*)&addr, sizeof(addr));
    if (nbytes != sizeof(frame)) {
      perror("CAN send error");
    }
    else
    {
      // // ===== 调试输出 =====
      // std::cout << "Send CAN Frame | ID: 0x" 
      //           << std::hex << std::uppercase << can_id 
      //           << " | Data: {";

      // for (int i = 0; i < len; ++i) {
      //     std::cout << "0x"
      //               << std::setw(2)
      //               << std::setfill('0')
      //               << static_cast<int>(data[i]);
      //     if (i != len - 1) std::cout << ", ";
      // }
      // std::cout << "}" << std::dec << std::endl;
      // // ===================
    }
    // 添加 10 毫秒的延迟
    usleep(10000); 
  }

  // 仿真模式下发布速度命令
  void publishWheelSpeedCommand(int left_rpm, int right_rpm)
  {
    if (!sim_mode_) {
      return;
    }
    std_msgs::Float64MultiArray msg;
    msg.data.clear();
    msg.data.push_back(static_cast<double>(left_rpm));
    msg.data.push_back(static_cast<double>(right_rpm));
    wheel_speed_pub_.publish(msg);
  }

  // 停止电机
  void stopWheelCommand()
  {
    if (sim_mode_) {
      publishWheelSpeedCommand(0, 0);
      return;
    }
    if(current_mode_ == 0) sendZeroSpeed();
    else sendstop();
  }

  // 下列三个函数用于生成CAN协议中的速度和位置命令
  static void int32ToCanData(int32_t value, unsigned char* data, int start_index)
  {
    data[start_index] = value & 0xFF;
    data[start_index + 1] = (value >> 8) & 0xFF;
    data[start_index + 2] = (value >> 16) & 0xFF;
    data[start_index + 3] = (value >> 24) & 0xFF;
  }
  void makeSpeedCommand(unsigned char* data, int rpm)
  {
    int32_t velocity = rpmToVelocity(rpm);
    data[0] = 0x23;
    data[1] = 0xFF;
    data[2] = 0x60;
    data[3] = 0x00;
    int32ToCanData(velocity, data, 4);
  }
  void makePositionCommand(unsigned char* data, int32_t position)
  {
    data[0] = 0x23;
    data[1] = 0x7A;
    data[2] = 0x60;
    data[3] = 0x00;
    int32ToCanData(position, data, 4);
  }

  // 发送复位和远程控制命令
  void sendResetAndRemoteControl()
  {
    unsigned char reset_data[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char remote_control_data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    sendCanFrame(sock_, addr_, 0x0000, reset_data);
    sendCanFrame(sock_, addr_, 0x0000, remote_control_data);
  }

  void sendVelocitySetup(int can_id)
  {
    unsigned char cw_06[8] = {0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
    unsigned char cw_07[8] = {0x2b, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00};
    unsigned char cw_0f[8] = {0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00};
    unsigned char speed_mode[] = {0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00};// 速度控制模式03
    unsigned char acc[8] = {0x23, 0x83, 0x60, 0x00, 0x60, 0xE3, 0x16, 0x00};
    unsigned char dec[8] = {0x23, 0x84, 0x60, 0x00, 0x60, 0xE3, 0x16, 0x00};

    sendCanFrame(sock_, addr_, can_id, cw_06);
    sendCanFrame(sock_, addr_, can_id, cw_07);
    sendCanFrame(sock_, addr_, can_id, cw_0f);
    sendCanFrame(sock_, addr_, can_id, speed_mode);
    sendCanFrame(sock_, addr_, can_id, acc);
    sendCanFrame(sock_, addr_, can_id, dec);
  }

  void sendPositionSetup(int can_id)
  {
    unsigned char cw_06[8] = {0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
    unsigned char cw_07[8] = {0x2b, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00};
    unsigned char cw_0f[8] = {0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00};
    unsigned char position_mode[8] = {0x2f, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00};
    unsigned char acc[8] = {0x23, 0x83, 0x60, 0x00, 0xc0, 0xd4, 0x01, 0x00};
    unsigned char dec[8] = {0x23, 0x84, 0x60, 0x00, 0xc0, 0xd4, 0x01, 0x00};
    unsigned char speed[] = {0x23, 0x81, 0x60, 0x00, 0xc0, 0xd4, 0x01, 0x00};

    sendCanFrame(sock_, addr_, can_id, cw_06);
    sendCanFrame(sock_, addr_, can_id, cw_07);
    sendCanFrame(sock_, addr_, can_id, cw_0f);
    sendCanFrame(sock_, addr_, can_id, position_mode);
    sendCanFrame(sock_, addr_, can_id, acc);
    sendCanFrame(sock_, addr_, can_id, dec);
    sendCanFrame(sock_, addr_, can_id, speed);
  }

  void sendPositionMove(int can_id, int32_t target_position)
  {
    unsigned char position_data[8] = {0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
    makePositionCommand(position_data, target_position);
    unsigned char relative_mode[8] = {0x2b, 0x40, 0x60, 0x00, 0x4f, 0x00, 0x00, 0x00};
    unsigned char start_move[8] = {0x2b, 0x40, 0x60, 0x00, 0x5f, 0x00, 0x00, 0x00};

    sendCanFrame(sock_, addr_, can_id, position_data);
    sendCanFrame(sock_, addr_, can_id, relative_mode);
    sendCanFrame(sock_, addr_, can_id, start_move);
  }

  void drainCanSocket()
  {
    struct can_frame frame;
    while (recvfrom(sock_, &frame, sizeof(frame), MSG_DONTWAIT, nullptr, nullptr) > 0) {
    }
  }

  bool readStatusResponse(int response_can_id, int16_t& status, double timeout_sec)
  {
    const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);

    while (ros::ok() && ros::Time::now() < deadline) {
      struct can_frame frame;
      int nbytes = recvfrom(sock_, &frame, sizeof(frame), MSG_DONTWAIT, nullptr, nullptr);

      if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          usleep(1000);
          continue;
        }
        perror("CAN status read error");
        return false;
      }

      if (nbytes == sizeof(frame) &&
          frame.can_id == static_cast<canid_t>(response_can_id) &&
          frame.can_dlc >= 6 &&
          frame.data[1] == 0x41 &&
          frame.data[2] == 0x60) {
        status = static_cast<int16_t>(frame.data[4] | (frame.data[5] << 8));
        return true;
      }
    }

    return false;
  }

  bool queryMotorStatus(int command_can_id, int response_can_id, int16_t& status)
  {
    unsigned char status_request[8] = {0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
    sendCanFrame(sock_, addr_, command_can_id, status_request);
    return readStatusResponse(response_can_id, status, 0.02);
  }

  static bool isTargetReached(int16_t status)
  {
    return (status & (1 << 10)) != 0;
  }

  static bool isFault(int16_t status)
  {
    return (status & (1 << 3)) != 0;
  }

  void sendZeroSpeed()
  {
    unsigned char stop_data[8];
    makeSpeedCommand(stop_data, 0);
    sendCanFrame(sock_, addr_, 0x601, stop_data);
    sendCanFrame(sock_, addr_, 0x602, stop_data);
  }

  void sendstop()
  {
    unsigned char halt0[8] = {0x2b, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00}; // Halt / Quick Stop
    unsigned char halt1[8] = {0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00}; // Halt / Quick Stop
    sendCanFrame(sock_, addr_, 0x601, halt0);
    sendCanFrame(sock_, addr_, 0x601, halt1);
    sendCanFrame(sock_, addr_, 0x602, halt0);
    sendCanFrame(sock_, addr_, 0x602, halt1);
    position_mode_initialized_ = false;
  }

  void ensureSpeedMode()
  {
    if (sim_mode_) {
      if (!speed_mode_initialized_) {
        ROS_INFO("[%s] Initializing SIM speed mode", action_name_.c_str());
        publishWheelSpeedCommand(0, 0);
        speed_mode_initialized_ = true;
      }
      current_mode_ = 0;
      position_mode_initialized_ = false;
      return;
    }

    if (current_mode_ != 0 || !speed_mode_initialized_) {
      ROS_INFO("[%s] Initializing speed control mode", action_name_.c_str());
      sendResetAndRemoteControl();
      sendVelocitySetup(0x601);
      sendVelocitySetup(0x602);
      current_mode_ = 0;
      stopWheelCommand();
      speed_mode_initialized_ = true;
      position_mode_initialized_ = false;
    }
  }

  void ensurePositionMode()
  {
    if (current_mode_ != 1 || !position_mode_initialized_) {
      sendResetAndRemoteControl();
      sendPositionSetup(0x601);
      sendPositionSetup(0x602);
      current_mode_ = 1;
      position_mode_initialized_ = true;
      speed_mode_initialized_ = false;
    }
  }

  void executeCB(const car_ctl::CarWheelMotionGoalConstPtr& goal)
  {
    car_ctl::CarWheelMotionResult result;
    car_ctl::CarWheelMotionFeedback feedback;

    if (goal->control_mode == 0) {
      ROS_INFO("[%s] Velocity control goal received: left=%.2f rpm, right=%.2f rpm, sim_mode=%s", action_name_.c_str(), goal->left_value, goal->right_value, sim_mode_ ? "true" : "false");
      ensureSpeedMode();

      unsigned char left_data[8];
      unsigned char right_data[8];
      int left_rpm = static_cast<int>(goal->left_value);
      int right_rpm = static_cast<int>(goal->right_value);
      makeSpeedCommand(left_data, left_rpm);
      makeSpeedCommand(right_data, right_rpm);

      ros::Rate rate(speed_publish_hz_);
      ros::Time start = ros::Time::now();
      double timeout = goal->timeout;
      bool timed_goal = timeout > 0.0;
      feedback.progress = 0.0;
      as_.publishFeedback(feedback);

      while (ros::ok() && as_.isActive() && !as_.isPreemptRequested() && !stop_requested_.load()) {
        if (sim_mode_) {
          publishWheelSpeedCommand(left_rpm, right_rpm);
        } else {
          sendCanFrame(sock_, addr_, 0x601, left_data);
          sendCanFrame(sock_, addr_, 0x602, right_data);
        }

        if (timed_goal) {
          double elapsed = (ros::Time::now() - start).toSec();
          feedback.progress = static_cast<float>(std::min(elapsed / timeout, 1.0));
          as_.publishFeedback(feedback);
          if (elapsed >= timeout) {
            break;
          }
        }

        rate.sleep();
      }

      stopWheelCommand();

      if (as_.isPreemptRequested() || stop_requested_.load()) {
        result.success = false;
        result.message = "Velocity goal preempted";
        as_.setPreempted(result);
        return;
      }

      result.success = true;
      result.message = "Velocity goal completed";
      as_.setSucceeded(result);
      return;
    }

    if (goal->control_mode == 1) {
      ROS_INFO("[%s] Position control goal received: left=%.2f, right=%.2f", action_name_.c_str(), goal->left_value, goal->right_value);
      if (sim_mode_) {
        result.success = false;
        result.message = "Position control not supported in simulation mode";
        as_.setAborted(result);
        return;
      }

      ROS_INFO("[%s] Initializing position control mode", action_name_.c_str());
      ensurePositionMode();
      ROS_INFO("[%s] Initialized", action_name_.c_str());

      // goal中的值代表电机转的圈数（revolutions），需要转换为对应的脉冲数（pulses）来发送给电机控制器
      int32_t left_target = static_cast<int32_t>(goal->left_value * pulse_per_rev_ * gear_ratio_);
      int32_t right_target = static_cast<int32_t>(goal->right_value * pulse_per_rev_ * gear_ratio_);

      drainCanSocket();
      sendPositionMove(0x601, left_target);
      sendPositionMove(0x602, right_target);

      ros::Rate rate(20);
      ros::Time start = ros::Time::now();
      bool timed_goal = goal->timeout >= 0.0;
      double timeout = goal->timeout > 0.0 ? goal->timeout : position_timeout_;
      feedback.progress = 0.0;
      as_.publishFeedback(feedback);

      while (ros::ok() && as_.isActive() && !as_.isPreemptRequested() && !stop_requested_.load()) {
        double elapsed = (ros::Time::now() - start).toSec();
        if (timed_goal && elapsed >= timeout) {
          stopWheelCommand();
          result.success = false;
          result.message = "Position goal timed out before target reached";
          as_.setAborted(result);
          return;
        }

        int16_t left_status = 0;
        int16_t right_status = 0;
        bool left_status_ok = queryMotorStatus(0x601, 0x581, left_status);
        bool right_status_ok = queryMotorStatus(0x602, 0x582, right_status);

        if ((left_status_ok && isFault(left_status)) || (right_status_ok && isFault(right_status))) {
          stopWheelCommand();
          result.success = false;
          result.message = "Position goal failed: motor fault";
          as_.setAborted(result);
          return;
        }

        if (left_status_ok && right_status_ok &&
            isTargetReached(left_status) && isTargetReached(right_status)) {
          feedback.progress = 1.0f;
          as_.publishFeedback(feedback);
          result.success = true;
          result.message = "Position goal reached";
          as_.setSucceeded(result);
          return;
        }

        if (timed_goal && timeout > 0.0) {
          feedback.progress = static_cast<float>(std::min(elapsed / timeout, 1.0));
          as_.publishFeedback(feedback);
        }
        rate.sleep();
      }

      if (as_.isPreemptRequested() || stop_requested_.load()) {
        stopWheelCommand();
        result.success = false;
        result.message = "Position goal preempted";
        as_.setPreempted(result);
        return;
      }

      stopWheelCommand();
      result.success = false;
      result.message = "Position goal interrupted before target reached";
      as_.setAborted(result);
      return;
    }

    result.success = false;
    result.message = "Unsupported control_mode";
    as_.setAborted(result);
  }
};

std::atomic<bool> g_shutdown_requested(false);

void sigintHandler(int)
{
  g_shutdown_requested.store(true);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "car_wheel_action_server", ros::init_options::NoSigintHandler);
  std::signal(SIGINT, sigintHandler);
  std::signal(SIGTERM, sigintHandler);

  CarWheelMotionActionServer server("car_wheel_motion");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Rate rate(20);
  while (ros::ok() && !g_shutdown_requested.load()) {
    rate.sleep();
  }

  server.emergencyStop();
  ros::shutdown();
  spinner.stop();
  return 0;
}
