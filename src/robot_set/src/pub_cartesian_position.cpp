// 绘制一个平面三角形图案（TCP位姿）

#include <ros/ros.h>
// #include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <robot_set/TCPState.h>

double freq = 10;

// 线性插值函数：从 start 到 end 插值 N 段（含首尾）
std::vector<std::vector<double>> interpolate(const std::vector<double>& start, const std::vector<double>& end, int num_points)
{
    std::vector<std::vector<double>> result;
    result.reserve(num_points);

    for (int i = 0; i < num_points; ++i)
    {
        double t = static_cast<double>(i) / (num_points - 1);
        std::vector<double> p(6);
        for (int j = 0; j < 6; ++j)
            p[j] = start[j] + t * (end[j] - start[j]);
        result.push_back(p);
    }
    return result;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cartesian_traj_publisher");
    ros::NodeHandle nh;

    ros::Publisher pose_pub = nh.advertise<robot_set::TCPState>("/cartesian_pose", 10);
    ros::Rate loop_rate(freq); //频率

    // 三角形顶点 (单位: m, rad)
    std::vector<std::vector<double>> vertices = {
        {0.3, 0.0, 0.2, 0.0, 0.0, 0.0},
        {0.4, 0.1, 0.2, 0.0, 0.0, 0.0},
        {0.2, 0.1, 0.2, 0.0, 0.0, 0.0}
    };

    // 拼接轨迹段
    std::vector<std::vector<double>> traj;
    int interp_points = 15; // 每条边插值点数

    for (size_t i = 0; i < vertices.size(); ++i)
    {
        size_t next = (i + 1) % vertices.size();
        auto segment = interpolate(vertices[i], vertices[next], interp_points);
        traj.insert(traj.end(), segment.begin(), segment.end());
    }

    size_t index = 0;
    while (ros::ok())
    {
        robot_set::TCPState pose_msg;
        pose_msg.position = traj[index];
        pose_msg.header.stamp = ros::Time::now();
        pose_pub.publish(pose_msg);
        ROS_INFO_STREAM("Published pose: [" 
            << pose_msg.position[0] << ", "
            << pose_msg.position[1] << ", "
            << pose_msg.position[2] << ", "
            << pose_msg.position[3] << ", "
            << pose_msg.position[4] << ", "
            << pose_msg.position[5] << "]");

        index = (index + 1) % traj.size();
        // ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
