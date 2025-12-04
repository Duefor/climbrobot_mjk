#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

double gyr_n = 1.9513416559e-03;   // 你的标定参数
double acc_n = 1.8774829775e-02;

double gyr_var = gyr_n * gyr_n;
double acc_var = acc_n * acc_n;

ros::Publisher pub;

void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
    sensor_msgs::Imu out = *msg;

    // 写入方差（variance = noise^2）
    out.angular_velocity_covariance[0] = gyr_var;
    out.angular_velocity_covariance[4] = gyr_var;
    out.angular_velocity_covariance[8] = gyr_var;

    out.linear_acceleration_covariance[0] = acc_var;
    out.linear_acceleration_covariance[4] = acc_var;
    out.linear_acceleration_covariance[8] = acc_var;

    pub.publish(out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_cov_republisher");
    ros::NodeHandle nh;

    pub = nh.advertise<sensor_msgs::Imu>("/camera/imu_calib", 50);
    ros::Subscriber sub = nh.subscribe("/camera/imu", 50, imuCallback);

    setlocale(LC_ALL, "");
    ROS_INFO("IMU转换节点已启动");

    ros::spin();
    return 0;
}
