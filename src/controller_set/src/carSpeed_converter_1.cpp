
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>

class carSpeed_converter
{
private:
    ros::NodeHandle nh_;
    // 安全限制
    double heading_est = 0.0;     // 实时朝向
    double current_heading;
    ros::Time last_time;
    double limit = 60.0;

    // 前进 / 后退
    double FORWARD_MIN = 30.0;
    double FORWARD_MAX = 105.0;
    double BACKWARD_MIN = 0.0;
    double BACKWARD_MAX = 14.0;

    double TURN_MAX = 30.0;

    double MAX_FRONT_SPEED = 1000.0;
    double MAX_BACK_SPEED  = 500.0;
    double MAX_TURN_SPEED  = 500.0;



    // 横移参数
    double DEADZONE = 30.0;   // 横移触发

    double SIDE_TURN_SPEED = 600.0;
    double SIDE_FORWARD_SPEED = 500.0;


    ros::Subscriber touch_joint_sub;
    ros::Publisher  car_speed_pub;
    ros::Subscriber imu_sub;

    Eigen::Vector3d acc_base{0,0,0};
    Eigen::Vector3d gyro_base{0,0,0};

    bool imu_ok = false;

    // 超时
    ros::Time last_msg_time;
    double INPUT_TIMEOUT = 0.3;

    enum SideState
    {
        IDLE,
        TURN,
        FORWARD,
        TURN_BACK
    };

    SideState side_state = IDLE;
    ros::Time state_start;

    int side_dir = 0;

public:
    carSpeed_converter(ros::NodeHandle& nh);
    ~carSpeed_converter();
    void publish_speed(double v,double w);
    void cb(const sensor_msgs::JointState::ConstPtr& msg);
    double computeHeading(const Eigen::Vector3d& acc);
    double updateHeading(double heading_est,
                     const Eigen::Vector3d& acc,
                     const Eigen::Vector3d& gyro,
                     double dt);
    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
};

carSpeed_converter::carSpeed_converter(ros::NodeHandle& nh)
{
    nh_ = nh;
    car_speed_pub  = nh_.advertise<std_msgs::Float64MultiArray>("/wheel_speed_cmd",10);
    touch_joint_sub = nh_.subscribe("/phantom/joint_states_car",10,&carSpeed_converter::cb,this);
    imu_sub = nh_.subscribe("/car/imu",10,&carSpeed_converter::imu_cb,this);
    
    last_time = ros::Time::now();
    last_msg_time = ros::Time::now();
}

carSpeed_converter::~carSpeed_converter()
{
}

void carSpeed_converter::publish_speed(double v,double w)
{
    std_msgs::Float64MultiArray msg;
    msg.data.resize(2);

    std::cout << heading_est << std::endl;

    // 角度限制
    if(heading_est > limit && w > 0) w = 0;
    if(heading_est < -limit && w < 0) w = 0;

    msg.data[0] = v - w;
    msg.data[1] = v + w;

    car_speed_pub.publish(msg);
}

void carSpeed_converter::cb(const sensor_msgs::JointState::ConstPtr& msg)
{
    if((ros::Time::now() - last_msg_time).toSec() > INPUT_TIMEOUT)
    {
        side_state = IDLE;
    }
    last_msg_time = ros::Time::now();
    if(msg->position.size() < 4)
        return;

    double axis1 = msg->position[0] * 180.0 / M_PI;
    double axis2 = msg->position[1] * 180.0 / M_PI;
    double axis4 = (msg->position[3]-3.1414822448183592) * 180.0 / M_PI;

    ros::Time now = ros::Time::now();

    double dt = (now - last_time).toSec();
    last_time = now;
    if(dt > INPUT_TIMEOUT) return;
    
    if(imu_ok)
    {
        heading_est = updateHeading(heading_est, acc_base, gyro_base, dt);
    }

    double v = 0;
    double w = 0;

    // 判断横移触发
    bool side_cmd = std::fabs(axis1) > DEADZONE;

    if(side_cmd)
    {
        side_dir = axis1 > 0 ? 1 : -1;
    }

    // ---------- 横移状态机 ----------
    if(side_state != IDLE)
    {
        double t = (now - state_start).toSec();

        if(side_state == TURN)
        {
            double target = current_heading + side_dir * 90.0;
            double error = target - heading_est;

            w = side_dir * SIDE_TURN_SPEED;

            if(std::fabs(error) < 2.0)
            {
                side_state = FORWARD;
                state_start = now;
            }
        }

        else if(side_state == FORWARD)
        {
            v = SIDE_FORWARD_SPEED;

            if(!side_cmd)
            {
                side_state = TURN_BACK;
                state_start = now;
            }
        }

        else if(side_state == TURN_BACK)
        {
            double error = current_heading - heading_est;
            w = -side_dir * SIDE_TURN_SPEED;
            if(std::fabs(error) < 2.0)
            {
                side_state = IDLE;
            }
        }


        publish_speed(v,w);
        return;
    }

    // ---------- 触发横移 ----------
    if(side_cmd)
    {
        side_state = TURN;
        state_start = now;
        current_heading = heading_est;  // 记录横移前的位置
        return;
    }

    // ---------- 前后 ----------

    // if(std::fabs(axis2) > DEADZONE)
    // {
    //     double scale = std::min(std::fabs(axis2)/60.0,1.0);

    //     if(axis2 > 0)
    //         v = scale * MAX_FRONT_SPEED;
    //     else
    //         v = -scale * MAX_BACK_SPEED;
    // }

    if (axis2 > FORWARD_MIN)
    {
        double scale = (axis2 - FORWARD_MIN) / (FORWARD_MAX - FORWARD_MIN);
        scale = std::min(scale, 1.0);

        v = -scale * MAX_FRONT_SPEED;
    }
    else if (axis2 < BACKWARD_MAX)
    {
        double scale = (BACKWARD_MAX - axis2) / (BACKWARD_MAX - BACKWARD_MIN);
        scale = std::min(scale, 1.0);

        v = scale * MAX_BACK_SPEED;
    }

    // ---------- 原地旋转 ----------
    if(std::fabs(axis4) > TURN_MAX)
    {
        double scale = std::min(std::fabs(axis4)/60.0,1.0);

        w = scale * MAX_TURN_SPEED;

        if(axis4 > 0)
            w = -w;
    }


    publish_speed(v,w);
}

double carSpeed_converter::computeHeading(const Eigen::Vector3d& acc)
{
    Eigen::Vector3d g = acc.normalized();
    Eigen::Vector3d head(1, 0, 0);  // 车头方向,以车头（机械臂基坐标系）x轴朝前为准

    double cos_theta = g.dot(head);
    cos_theta = std::clamp(cos_theta, -1.0, 1.0);

    double theta = acos(cos_theta) * 180.0 / M_PI;

    // 符号（左右）
    double sign = (g.cross(head)).z() > 0 ? 1.0 : -1.0;

    return sign * theta;
}

double carSpeed_converter::updateHeading(double heading_est,
                     const Eigen::Vector3d& acc,
                     const Eigen::Vector3d& gyro,
                     double dt)
{
    // gyro预测（注意轴要根据你实际情况调整）
    double heading_gyro = heading_est + gyro.z() * dt * 180.0 / M_PI;

    // acc角度
    double heading_acc = computeHeading(acc);
    std::cout << "当前acc角度：" << heading_acc << std::endl;

    // 判断是否静止（抗干扰）
    double norm = acc.norm();

    if(std::fabs(norm - 9.81) < 0.5)
    {
        double alpha = 0.02;
        return (1 - alpha) * heading_gyro + alpha * heading_acc;
    }
    else
    {
        return heading_gyro;
    }
}

void carSpeed_converter::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    acc_base << msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z;

    gyro_base << msg->angular_velocity.x,
                 msg->angular_velocity.y,
                 msg->angular_velocity.z;

    imu_ok = true;
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"carSpeed_converter");
    ros::NodeHandle nh;
    carSpeed_converter converter(nh);
    

    ros::spin();
}