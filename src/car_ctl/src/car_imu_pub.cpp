#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

class ImuKinematics
{
public:
    ImuKinematics()
    {
        sub_imu_ = nh_.subscribe("/camera/imu", 10, &ImuKinematics::imuCallback, this);
        sub_joint_ = nh_.subscribe("/cs66/joint_states", 10, &ImuKinematics::jointCallback, this);
        pub_imu_ = nh_.advertise<sensor_msgs::Imu>("/car/imu", 10);

        joint_positions_.resize(DOF, 0.0);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_joint_;
    ros::Publisher pub_imu_;

    std::vector<double> joint_positions_;

    // ====== 需要你填写 ======
    static const int DOF = 6;

    // DH参数（你填）
    std::vector<double> a_     = {0, 0, -0.427, -0.3905, 0, 0};
    std::vector<double> d_     = {0.1625, 0, 0, 0.1475, 0.0965, 0.092};
    std::vector<double> alpha_ = {0, M_PI/2, 0, 0, M_PI/2, -M_PI/2};

    // 相机相对于第5关节的变换（你填）
    Matrix4d T_joint5_cam = Matrix4d::Identity();

    // 误差旋转矩阵
    Matrix3d R_err = Matrix3d::Identity();

    // =======================

    void jointCallback(const sensor_msgs::JointStateConstPtr& msg)
    {
        for (size_t i = 0; i < DOF && i < msg->position.size(); i++)
        {
            joint_positions_[i] = msg->position[i];
        }
    }

    // DH变换
    Matrix4d dh(double a, double d, double alpha, double theta)
    {
        double ca = cos(alpha);
        double sa = sin(alpha);
        double ct = cos(theta);
        double st = sin(theta);
        Matrix4d T;
        T << ct, -st, 0, a,
             st*ca, ct*ca, -sa, -sa*d,
             st*sa, ct*sa, ca,  ca*d,
             0, 0, 0, 1;

        return T;
    }

    // base -> joint5
    Matrix4d forwardKinematicsToJoint5()
    {
        Matrix4d T = Matrix4d::Identity();

        for (int i = 0; i < 5; i++)
        {
            T = T * dh(a_[i], d_[i], alpha_[i], joint_positions_[i]);
        }

        return T;
    }

    void imuCallback(const sensor_msgs::ImuConstPtr& msg)
    {
        // Step 1: base -> joint5
        Matrix4d T_base_j5 = forwardKinematicsToJoint5();

        // Step 2: joint5 -> camera（已知）
        Matrix4d T_j5_cam = T_joint5_cam;

        // Step 3: base -> camera
        Matrix4d T_base_cam = T_base_j5 * T_j5_cam;

        // Step 4: 取旋转
        Matrix3d R_base_cam_origin = T_base_cam.block<3,3>(0,0);

        // 修正
        Matrix3d R_base_cam = R_err * R_base_cam_origin;

        // ====== IMU数据 ======
        Vector3d acc_cam(msg->linear_acceleration.x,
                         msg->linear_acceleration.y,
                         msg->linear_acceleration.z);

        Vector3d gyro_cam(msg->angular_velocity.x,
                          msg->angular_velocity.y,
                          msg->angular_velocity.z);

        // Step 5: 转到 base
        Vector3d acc_base = R_base_cam * acc_cam;
        Vector3d gyro_base = R_base_cam * gyro_cam;

        // ====== 输出 ======
        sensor_msgs::Imu imu_out = *msg;
        imu_out.header.frame_id = "base_link";

        imu_out.linear_acceleration.x = acc_base.x();
        imu_out.linear_acceleration.y = acc_base.y();
        imu_out.linear_acceleration.z = acc_base.z();

        imu_out.angular_velocity.x = gyro_base.x();
        imu_out.angular_velocity.y = gyro_base.y();
        imu_out.angular_velocity.z = gyro_base.z();

        // ====== orientation（可选）=====
        if (!(msg->orientation_covariance[0] == -1))
        {
            Quaterniond q_cam(
                msg->orientation.w,
                msg->orientation.x,
                msg->orientation.y,
                msg->orientation.z);

            Quaterniond q_base(R_base_cam * q_cam.toRotationMatrix());

            imu_out.orientation.w = q_base.w();
            imu_out.orientation.x = q_base.x();
            imu_out.orientation.y = q_base.y();
            imu_out.orientation.z = q_base.z();
        }

        pub_imu_.publish(imu_out);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_forward_kinematics");
    ImuKinematics node;
    ros::spin();
    return 0;
}