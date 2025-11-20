#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_to_camera_node");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Publisher pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("camera_pose_in_map", 10);

    ros::Rate rate(30);  // 30Hz 更新 Unity 足够

    while (ros::ok())
    {
        try
        {
            // 1. 读取 map→odom
            geometry_msgs::TransformStamped map_to_odom =
                tfBuffer.lookupTransform("map", "odom", ros::Time(0));

            // 2. 读取 odom→camera_link
            geometry_msgs::TransformStamped odom_to_cam =
                tfBuffer.lookupTransform("odom", "camera_link", ros::Time(0));

            // 3. 组合变换：map→camera = map→odom * odom→camera
            geometry_msgs::TransformStamped map_to_cam;
            tf2::doTransform(odom_to_cam, map_to_cam, map_to_odom);

            // 4. 转成 Pose
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";

            pose.pose.position.x = map_to_cam.transform.translation.x;
            pose.pose.position.y = map_to_cam.transform.translation.y;
            pose.pose.position.z = map_to_cam.transform.translation.z;

            pose.pose.orientation = map_to_cam.transform.rotation;

            // 5. 发布或发送给 Unity
            pose_pub.publish(pose);
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("TF unavailable: %s", ex.what());
        }

        rate.sleep();
    }

    return 0;
}
