#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_tf_static_pub");

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    std::vector<geometry_msgs::TransformStamped> transforms;

    // camera_link -> camera_depth_frame
    {
        geometry_msgs::TransformStamped t;
        t.header.stamp = ros::Time::now();
        t.header.frame_id = "camera_link";
        t.child_frame_id = "camera_depth_frame";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;
        transforms.push_back(t);
    }

    // camera_depth_frame -> camera_depth_optical_frame
    {
        geometry_msgs::TransformStamped t;
        t.header.stamp = ros::Time::now();
        t.header.frame_id = "camera_depth_frame";
        t.child_frame_id = "camera_depth_optical_frame";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = -0.5;
        t.transform.rotation.y = 0.5;
        t.transform.rotation.z = -0.5;
        t.transform.rotation.w = 0.5;
        transforms.push_back(t);
    }

    // camera_link -> camera_color_frame
    {
        geometry_msgs::TransformStamped t;
        t.header.stamp = ros::Time::now();
        t.header.frame_id = "camera_link";
        t.child_frame_id = "camera_color_frame";
        t.transform.translation.x = -0.0001909840357;
        t.transform.translation.y = 0.0149448504671;
        t.transform.translation.z = -0.0000926668290;
        t.transform.rotation.x = -0.0000390051755;
        t.transform.rotation.y = -0.0008308666875;
        t.transform.rotation.z = 0.0040108575486;
        t.transform.rotation.w = 0.9999915957451;
        transforms.push_back(t);
    }

    // camera_color_frame -> camera_color_optical_frame
    {
        geometry_msgs::TransformStamped t;
        t.header.stamp = ros::Time::now();
        t.header.frame_id = "camera_color_frame";
        t.child_frame_id = "camera_color_optical_frame";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = -0.5;
        t.transform.rotation.y = 0.5;
        t.transform.rotation.z = -0.5;
        t.transform.rotation.w = 0.5;
        transforms.push_back(t);
    }

    // base_link -> camera_link  (假设相机在机器人正前方，无位移)
    {
        geometry_msgs::TransformStamped t;
        t.header.stamp = ros::Time::now();
        t.header.frame_id = "base_link";
        t.child_frame_id = "camera_link";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;
        transforms.push_back(t);
    }

    // 一次性发布所有 /tf_static
    static_broadcaster.sendTransform(transforms);

    ros::spin();
    return 0;
}
