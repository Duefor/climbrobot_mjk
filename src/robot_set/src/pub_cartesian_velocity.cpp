// 绘制一个三角形

#include <ros/ros.h>
// #include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <cmath>
#include <robot_set/TCPState.h>
#include <sensor_msgs/JointState.h>

double freq = 10;

// 线性插值：t in [0,1]
inline void lerp(double x0, double y0, double x1, double y1, double t, double &x, double &y) {
    x = x0 + t * (x1 - x0);
    y = y0 + t * (y1 - y0);
}

// 根据 phase（范围 [0, cycle_time)）返回当前末端位置 (x,y,z)
void posFromPhase(double phase, double cycle_time, const std::vector<std::pair<double,double>>& verts,
                  double &x, double &y, double &z)
{
    double seg_time = cycle_time / 3.0;
    // 确保 phase 在 [0,cycle_time)
    while (phase < 0) phase += cycle_time;
    if (phase >= cycle_time) phase = fmod(phase, cycle_time);

    int seg = static_cast<int>(phase / seg_time);
    double local_t = (phase - seg * seg_time) / seg_time;
    int next_seg = (seg + 1) % 3;

    lerp(verts[seg].first, verts[seg].second,
         verts[next_seg].first, verts[next_seg].second,
         local_t, x, y);
    z = 0.3;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "triangle_motion_pub_fixed");
    ros::NodeHandle nh;

    ros::Publisher vel_pub = nh.advertise<robot_set::TCPState>("/cartesian_vel", 10);

    // 测试
    // ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    // sensor_msgs::JointState testmsg;
    // testmsg.position = {0.0,-0.93,-2.21,-1.58,1.42,0.0};

    ros::Rate loop_rate(freq); // 10 Hz
    const double dt = 0.1;  // 与 loop_rate 对应
    const double cycle_time = 6.0; // 一个三角形周期（秒）

    std::vector<std::pair<double,double>> vertices = {
        {0.2, 0.0},
        {0.0, 0.2},
        {-0.2, 0.0}
    };

    double start_time = ros::Time::now().toSec();

    while (ros::ok())
    {
        double now = ros::Time::now().toSec();
        double t = now - start_time;

        // 当前相位与前一相位（处理回绕）
        double phase_now = fmod(t, cycle_time);
        if (phase_now < 0) phase_now += cycle_time;
        double phase_prev = phase_now - dt;
        if (phase_prev < 0) phase_prev += cycle_time;

        // 计算当前与前一位置
        double x_now, y_now, z_now;
        double x_prev, y_prev, z_prev;
        posFromPhase(phase_now, cycle_time, vertices, x_now, y_now, z_now);
        posFromPhase(phase_prev, cycle_time, vertices, x_prev, y_prev, z_prev);

        // 差分速度
        double vx = (x_now - x_prev) / dt;
        double vy = (y_now - y_prev) / dt;
        double vz = (z_now - z_prev) / dt;


        // 发布末端笛卡尔速度 [vx, vy, vz, wx, wy, wz]
        robot_set::TCPState vel_msg;
        // vel_msg.velocity = {vx, vy, vz, 0.0, 0.0, 0.0};
        vel_msg.velocity = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
        vel_msg.header.stamp = ros::Time::now();
        vel_pub.publish(vel_msg);

        std::cout << "当前速度为：" << "[" << vel_msg.velocity[0] << "," << vel_msg.velocity[1] << "," << vel_msg.velocity[2] << "," 
            << vel_msg.velocity[3] << "," << vel_msg.velocity[4] << "," << vel_msg.velocity[5] << "]" << std::endl;

        // 发布测试关节角
        // joint_pub.publish(testmsg);

        // ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


