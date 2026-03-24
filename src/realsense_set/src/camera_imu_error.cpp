#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <robot_sdk_wrapper/robot_sdk.h>

Eigen::Matrix3d R_imu_meas = Eigen::Matrix3d::Identity();
bool imu_ready = false;


// 保存误差
std::vector<Eigen::Matrix3d> error_list;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    Eigen::Quaterniond q(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z
    );

    q.normalize();
    R_imu_meas = q.toRotationMatrix();
    imu_ready = true;
}

void printRP(const Eigen::Matrix3d& R, const std::string& name)
{
    double pitch = asin(-R(2,0));
    double roll  = atan2(R(2,1), R(2,2));

    std::cout << name << " | roll: " << roll * 180.0/M_PI
              << " deg, pitch: " << pitch * 180.0/M_PI << " deg" << std::endl;
}


std::vector<double> a_     = {0, 0, -0.427, -0.3905, 0, 0};
std::vector<double> d_     = {0.1625, 0, 0, 0.1475, 0.0965, 0.092};
std::vector<double> alpha_ = {0, M_PI/2, 0, 0, M_PI/2, -M_PI/2};
Eigen::Matrix4d dh(double a, double d, double alpha, double theta)
{
    double ca = cos(alpha);
    double sa = sin(alpha);
    double ct = cos(theta);
    double st = sin(theta);
    Eigen::Matrix4d T;
    T << ct, -st, 0, a,
            st*ca, ct*ca, -sa, -sa*d,
            st*sa, ct*sa, ca,  ca*d,
            0, 0, 0, 1;

    return T;
}

Eigen::Matrix3d forwardKinematics_R_base_j5(const std::vector<double>& q)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 5; i++)
    {
        T = T * dh(a_[i], d_[i], alpha_[i], q[i]);
    }
    return T.block<3,3>(0,0);
}


double rotationErrorAngle(const Eigen::Matrix3d& R)
{
    double trace = R.trace();
    double angle = acos((trace - 1.0) / 2.0);
    return angle * 180.0 / M_PI;
}

std::vector<double> getRobotJoint(EliteCSRobotSDK& robot)
{
    auto joint = robot.getCurrentJoint();

    std::vector<double> q(joint.begin(), joint.end());

    return q;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_check_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/imu", 10, imuCallback);

    EliteCSRobotSDK robot("robotip","pcip",true,"external_control.script","output_recipe.txt","input_recipe.txt");



    // ====== 固定外参（你要填）======

    // j5 -> cam（手眼标定）
    Eigen::Matrix3d R_j5_cam;
    R_j5_cam << 0.997269, -0.003001, -0.07379,
  0.073844,  0.053722, 0.995822, 
 0.000975, -0.998551,  0.053797  ;  

    // cam -> imu（realsense提供）
    Eigen::Matrix3d R_cam_imu;
    R_cam_imu.setIdentity();

    // world -> base
    Eigen::Matrix3d R_base_world;
    R_base_world.setIdentity();  // TODO: 替换

    ros::Rate rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        if (!imu_ready)
        {
            rate.sleep();
            continue;
        }

        // ===== 1. 读取关节角 =====
        std::vector<double> q = getRobotJoint(robot);

        // ===== 2. 正运动学 =====
        Eigen::Matrix3d R_base_j5 = forwardKinematics_R_base_j5(q);

        // ===== 3. 推算 imu 姿态（参考）=====
        Eigen::Matrix3d R_ref =
            R_base_j5 * R_j5_cam * R_cam_imu;

        // ===== 4. 实际 imu =====
        Eigen::Matrix3d R_meas = R_base_world * R_imu_meas; // 将相机imu转到base坐标系

        // ===== 5. 误差 =====
        Eigen::Matrix3d R_err = R_ref * R_meas.transpose();

        // 保存
        error_list.push_back(R_err);

        // ===== 打印 =====
        printRP(R_ref,  "REF");
        printRP(R_meas, "IMU");

        std::cout << "---- ERROR ----" << std::endl;
        printRP(R_err, "ERR");
        std::cout << "Angle Error: "
          << rotationErrorAngle(R_err)
          << " deg" << std::endl;

        std::cout << "==========================" << std::endl;

        rate.sleep();
    }

    return 0;
}