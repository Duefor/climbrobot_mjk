#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <algorithm>

// 安全限制
double heading_est = 0.0;     // 估计朝向
double current_heading;
ros::Time last_time;
double limit = 60.0;

// 前进 / 后退
constexpr double FORWARD_MIN = 30.0;
constexpr double FORWARD_MAX = 105.0;
constexpr double BACKWARD_MIN = 0.0;
constexpr double BACKWARD_MAX = 14.0;

constexpr double TURN_MAX = 30.0;

constexpr double MAX_FRONT_SPEED = 1000.0;
constexpr double MAX_BACK_SPEED  = 500.0;
constexpr double MAX_TURN_SPEED  = 500.0;



// 横移参数
constexpr double DEADZONE = 30.0;   // 横移触发

constexpr double SIDE_TURN_SPEED = 600.0;
constexpr double SIDE_FORWARD_SPEED = 500.0;

constexpr double SIDE_TURN_TIME = 0.5;
double SIDE_TURN_time = SIDE_TURN_TIME;


ros::Subscriber touch_joint_sub;
ros::Publisher  car_speed_pub;

// 超时
ros::Time last_msg_time;
constexpr double INPUT_TIMEOUT = 0.3;

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

void publish_speed(double v,double w,double dt)
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

    // 更新角度（用最终速度）
    heading_est += w * dt * 0.1;
    heading_est = std::clamp(heading_est,-90.0,90.0);
}

void cb(const sensor_msgs::JointState::ConstPtr& msg)
{
    if((ros::Time::now() - last_msg_time).toSec() > INPUT_TIMEOUT)
    {
        side_state = IDLE;
        SIDE_TURN_time = SIDE_TURN_TIME;
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
            w = side_dir * SIDE_TURN_SPEED;
            if((w > 0 && heading_est > limit) || (w < 0 && heading_est < -limit) || (t > SIDE_TURN_TIME))
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
            w = -side_dir * SIDE_TURN_SPEED;

            if((side_dir > 0 && heading_est < current_heading+2) || (side_dir < 0 && heading_est > current_heading-2))
            {
                side_state = IDLE;
            }
        }


        publish_speed(v,w,dt);
        return;
    }

    // ---------- 触发横移 ----------
    if(side_cmd)
    {
        side_state = TURN;
        state_start = now;
        current_heading = heading_est;
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


    publish_speed(v,w,dt);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"carSpeed_converter");
    ros::NodeHandle nh;

    car_speed_pub  = nh.advertise<std_msgs::Float64MultiArray>("/wheel_speed_cmd",10);
    touch_joint_sub = nh.subscribe("/phantom/joint_states_car",10,cb);
    
    last_time = ros::Time::now();
    last_msg_time = ros::Time::now();

    ros::spin();
}