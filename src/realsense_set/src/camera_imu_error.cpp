#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <vector>
#include <numeric>
#include <cmath>
#include <robot_sdk_wrapper/robot_sdk.h>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace Eigen;

int getKey()
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    newt.c_lflag &= ~(ICANON | ECHO); // 非阻塞
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    return ch;
}


Matrix3d computeRotationBias(
    const std::vector<Vector3d>& g_ref_list,
    const std::vector<Vector3d>& g_imu_list)
{
    Matrix3d H = Matrix3d::Zero();

    for (size_t i = 0; i < g_ref_list.size(); ++i)
    {
        Vector3d g_ref = g_ref_list[i];
        Vector3d g_imu = g_imu_list[i];

        H += g_imu * g_ref.transpose();
    }

    // SVD
    JacobiSVD<Matrix3d> svd(H, ComputeFullU | ComputeFullV);
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();

    Matrix3d R = U * V.transpose();

    // 保证 det = 1
    if (R.determinant() < 0)
    {
        Matrix3d U_fix = U;
        U_fix.col(2) *= -1;
        R = U_fix * V.transpose();
    }

    return R;
}


// ====== 参数 ======
const int SAMPLE_NUM = 200;      // 每次平均帧数
const int POSE_NUM   = 50;       // 采样姿态数

// ====== 全局变量 ======
std::vector<Vector3d> imu_buffer;
bool collecting = false;

// 手眼标定（你需要替换）
Matrix4d T_ee_cam;

// ====== 工具函数 ======
double computeAngle(const Vector3d& a, const Vector3d& b)
{
    double cos_theta = a.dot(b);
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    return std::acos(cos_theta) * 180.0 / M_PI;
}

// ====== 机械臂正运动学（你需要实现） ======
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

Matrix4d forwardKinematics(EliteCSRobotSDK& robot)
{
    Matrix4d T = Matrix4d::Identity();
    auto joint = robot.getCurrentJoint();
    std::vector<double> q(joint.begin(),joint.end());
    for (int i = 0; i < 5; i++)
    {
        T = T * dh(a_[i], d_[i], alpha_[i], q[i]);
    }
    return T;
}

// ====== IMU 回调 ======
void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
    if (!collecting) return;

    Vector3d acc(msg->linear_acceleration.x,
                 msg->linear_acceleration.y,
                 msg->linear_acceleration.z);

    imu_buffer.push_back(acc);
}

std::vector<Vector3d> g_ref_list;
std::vector<Vector3d> g_imu_list;
// ====== 主程序 ======
int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_gravity_eval");
    ros::NodeHandle nh;
    EliteCSRobotSDK robot("192.168.1.199","192.168.1.150",true,"external_control.script","/home/duefor/climbrobot_mjk/src/robot_sdk_wrapper/resource/output_recipe.txt","/home/duefor/climbrobot_mjk/src/robot_sdk_wrapper/resource/input_recipe.txt");
    robot.init_read_data();


    ros::Subscriber sub = nh.subscribe("/camera/imu", 1000, imuCallback);

    Matrix3d R_bias_0 = Matrix3d::Identity();
//     R_bias_0 << 
//     -0.714952,  -0.69014,  0.112026,
//  0.698833, -0.710372, 0.0836948,
// 0.0218192,  0.138125,  0.990174;

    // ===== 手眼标定（示例）=====
//     T_ee_cam << 
//      0.997269, -0.003001, -0.07379,  -0.014354,
//   0.073844,  0.053722 , 0.995822 , 0.033662,
//   0.000975, -0.998551,  0.053797 , 0.15485 ,
//   0 ,       0  ,      0    ,    1   ;   

    T_ee_cam << 
   0.99933871, -0.03441112, -0.01174839, -0.03749693,
  0.01460956,  0.08411249,  0.99634916, -0.02097434,
 -0.0332973,  -0.99586192,  0.0845596,   0.06478461,
  0,          0,          0,          1;        
    
    // TODO: 填入你的标定结果

    std::vector<double> error_list;

    ros::Rate rate(200);

    int pose_id = 0;

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();

        int key = getKey();

        // ===== 退出 =====
        if (key == 'q')
        {
            ROS_INFO("Exit.");
            break;
        }

        if (key == 's')
        {
            ROS_INFO("Move robot to pose %d and keep still...", pose_id);
            sleep(1);  // 等待稳定

            imu_buffer.clear();
            collecting = true;

            // 采集数据
            while (imu_buffer.size() < SAMPLE_NUM)
            {
                ros::spinOnce();
                rate.sleep();
            }

            collecting = false;

            // ===== 计算 IMU 重力 =====
            Vector3d g_imu = Vector3d::Zero();
            for (auto& v : imu_buffer)
                g_imu += v;
            g_imu /= imu_buffer.size();

            g_imu.normalize();
            g_imu = -g_imu;  // 方向修正
            g_imu = R_bias_0.transpose() * g_imu;

            // ===== 计算参考重力 =====
            Matrix4d T_base_ee = forwardKinematics(robot);
            Matrix4d T_base_cam = T_base_ee * T_ee_cam;

            Matrix3d R_base_cam = T_base_cam.block<3,3>(0,0);

            Vector3d g_base(0, 0, -1);
            Vector3d g_ref = R_base_cam.transpose() * g_base;
            g_ref.normalize();

            g_ref_list.push_back(g_ref);
            g_imu_list.push_back(g_imu);
            // ===== 误差 =====
            double error = computeAngle(g_ref, g_imu);
            error_list.push_back(error);

            ROS_INFO("Pose %d Error: %.4f deg", pose_id, error);
            Vector3d e = g_imu.cross(g_ref);
            // 转成角度（小角度近似）
            double err_roll  = e.x() * 180.0 / M_PI;
            double err_pitch = e.y() * 180.0 / M_PI;
            ROS_INFO("Pose %d: Roll Err = %.3f deg, Pitch Err = %.3f deg", pose_id, err_roll, err_pitch);
            ROS_INFO("----------------------------------------");
            pose_id++;
        }
    }

    // ===== 统计 =====
    double mean = std::accumulate(error_list.begin(), error_list.end(), 0.0) / error_list.size();

    double sq_sum = 0.0;
    for (auto e : error_list)
        sq_sum += (e - mean) * (e - mean);
    double stddev = std::sqrt(sq_sum / error_list.size());

    double max_err = *std::max_element(error_list.begin(), error_list.end());

    ROS_INFO("====== RESULT ======");
    ROS_INFO("Mean Error: %.4f deg", mean);
    ROS_INFO("Std  Error: %.4f deg", stddev);
    ROS_INFO("Max  Error: %.4f deg", max_err);


    ROS_INFO("====== ====== ======");
    Matrix3d R_bias = computeRotationBias(g_ref_list, g_imu_list);
    std::cout << "R_bias = \n" << R_bias << std::endl;

    return 0;
}