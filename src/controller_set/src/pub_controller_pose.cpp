#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"pub_pose");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("controller_pose",10);
    std_msgs::Float64MultiArray msg;
    msg.data = {1,1,1,1,1,1};
    ros::Rate rate(1);
    while (ros::ok())
    {
        pub.publish(msg);
        rate.sleep();
    }
    
    return 0;
}
