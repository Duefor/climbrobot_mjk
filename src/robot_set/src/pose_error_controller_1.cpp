#include <ros/ros.h>
// #include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <mutex>
#include <robot_set/TCPState.h>
#include <iostream>
#include <iomanip>

using namespace Eigen;

VectorXd actual_pose(6);  // 实际tcp位姿
VectorXd desired_pose(6); // 期望tcp位姿
VectorXd desired_vel(6);  // 期望tcp速度
VectorXd error_prev(6);
VectorXd error_integral(6);
MatrixXd K_p = MatrixXd::Identity(6,6); // 位姿误差增益矩阵：p
MatrixXd K_i = MatrixXd::Identity(6,6); // 位姿积分误差增益矩阵：i
MatrixXd K_d = MatrixXd::Identity(6,6); // 位姿微分误差增益矩阵：d
std::mutex mtx;


// 机械臂实际tcp位姿反馈
void actualCB(const robot_set::TCPState::ConstPtr &msg)
{
  if (msg->position.size() == 6)
  {
    std::lock_guard<std::mutex> lk(mtx);
    for (int i=0;i<6;i++) actual_pose[i]=msg->position[i];
  }
}

// 期望的机械臂tcp位姿
void desiredCB(const robot_set::TCPState::ConstPtr &msg)
{
  if (msg->position.size() == 6)
  {
    std::lock_guard<std::mutex> lk(mtx);
    for (int i=0;i<6;i++) desired_pose[i]=msg->position[i];
  }
}

// 期望的机械臂tcp速度
void velCB(const robot_set::TCPState::ConstPtr &msg)
{
  if (msg->velocity.size() == 6)
  {
    std::lock_guard<std::mutex> lk(mtx);
    for (int i=0;i<6;i++) desired_vel[i]=msg->velocity[i];
  }
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "pose_error_controller");
  ros::NodeHandle nh;

  ros::Subscriber sub_actual = nh.subscribe("/tcp_state", 1, actualCB);
  ros::Subscriber sub_desired = nh.subscribe("/desired_pose_sub", 1, desiredCB);
  ros::Subscriber sub_vel = nh.subscribe("/desired_vel_sub", 1, velCB);
  ros::Publisher pub_cmd = nh.advertise<robot_set::TCPState>("/cartesian_vel", 1);

  actual_pose.setZero();
  desired_pose.setZero();
  desired_vel.setZero();

  error_prev.setZero();
  error_integral.setZero();
// 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
// 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  // 临时pid参数
  K_p.diagonal() << 2.5, 2.5, 2.5, 2.0, 2.0, 2.0;
  K_i.diagonal() << 0.03, 0.03, 0.03, 0.02, 0.02, 0.02;
  K_d.diagonal() << 0.3, 0.3, 0.3, 0.2, 0.2, 0.2;

  VectorXd sign_correction(6);
  sign_correction << 1, 1, 1, -1, -1, 1;  // rx,ry方向相反

  double rrr = 50;
  ros::Rate rate(rrr);
  while (ros::ok())
  {
    ros::spinOnce();
    VectorXd a, d, v_d;
    {
      std::lock_guard<std::mutex> lk(mtx);
      a = actual_pose;
      d = desired_pose;
      v_d = desired_vel;
    }

    double dt = 1.0/rrr;
    
    auto angle_diff = [](double target, double current){
      double diff = target - current;
      return std::atan2(std::sin(diff), std::cos(diff)); // 归一化到 (-pi,pi]
    };

    VectorXd xi(6);
    xi.head<3>() = d.head<3>() - a.head<3>(); // 线性位置误差
    for (int i = 3; i < 6; ++i) {
      xi[i] = angle_diff(d[i], a[i]);
    }
    xi = sign_correction.asDiagonal() * xi;

    // std::cout<<xi<<std::endl;
    // std::cout<<"-----------------------------------------"<<std::endl;
    // std::cout << "当前位姿指令为：" << "[" << a[0] << "," << a[1] << "," << a[2] << "," 
    //     << a[3] << "," << a[4] << "," << a[5] << "]" << std::endl;
    auto xii = xi;
    for(int i = 0; i<6; ++i)
    {
      xii[i]=std::round(xii[i]*1e7)/1e7;
    }
    std::cout<<std::fixed<<std::setprecision(7);
    std::cout<<"原始误差:"<<std::endl<<xii;
    std::cout<<"-----------------------------------------"<<std::endl;
    

    // 积分项（带限幅防止积分饱和）
    error_integral += xi * dt;
    // 需保证 ki * integral_limit <= 机械臂笛卡尔最大速度
    double integral_limit = 2.0; // 可调
    for (int i = 0; i < 6; ++i)
    {
        error_integral[i] = std::clamp(error_integral[i], -integral_limit, integral_limit);
    }

    // 微分项
    static VectorXd error_derivative_filtered = VectorXd::Zero(6);
    double alpha = 0.1; // 滤波系数，越小越平滑
    VectorXd error_derivative = (xi - error_prev) / dt;
    error_derivative_filtered = alpha * error_derivative + (1 - alpha) * error_derivative_filtered;
    error_prev = xi;

    // PID 控制律
    VectorXd v_cmd = v_d + K_p * xi + K_i * error_integral + K_d * error_derivative_filtered;


    robot_set::TCPState msg;
    msg.velocity.resize(6);
    for (int i=0;i<6;i++) msg.velocity[i]=v_cmd[i];
    pub_cmd.publish(msg);
    // std::cout << "当前tcp速度控制指令为：" << "[" << msg.velocity[0] << "," << msg.velocity[1] << "," << msg.velocity[2] << "," 
    //     << msg.velocity[3] << "," << msg.velocity[4] << "," << msg.velocity[5] << "]" << std::endl;

    rate.sleep();
  }
  return 0;
}