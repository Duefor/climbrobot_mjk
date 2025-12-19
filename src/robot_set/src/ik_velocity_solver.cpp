#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <robot_set/TCPState.h>

using namespace Eigen;
using namespace std;

ros::Publisher pub;
sensor_msgs::JointState current_joint;
bool q_ready = false;

// DH 参数
const double d[6]     = {0.1625, 0, 0, 0.1475, 0.0965, 0.092};
const double a[6]     = {0, 0, -0.427, -0.3905, 0, 0};
const double alpha[6] = {0, M_PI/2, 0, 0, M_PI/2, -M_PI/2};

Vector3d so3Log(const Matrix3d &R)
{
    AngleAxisd aa(R);
    return aa.axis() * aa.angle();
}

// DH变换表
Matrix4d mdhTransform(double a, double alpha, double d, double theta)
{
    double ca = cos(alpha);
    double sa = sin(alpha);
    double ct = cos(theta);
    double st = sin(theta);

    Matrix4d T;
    // T << ct, -st*ca,  st*sa, a*ct,
    //      st,  ct*ca, -ct*sa, a*st,
    //       0,      sa,     ca,    d,
    //       0,       0,      0,    1;

    T << ct, -st, 0, a,
         st*ca, ct*ca, -sa, -sa*d,
         st*sa, ct*sa, ca,  ca*d,
         0, 0, 0, 1;


    return T;
}

// 正运动学
Matrix4d forwardKinematics(const vector<double>& q)
{
    Matrix4d T = Matrix4d::Identity();
    for (int i = 0; i < 6; ++i)
    {
        T *= mdhTransform(a[i], alpha[i], d[i], q[i]);
    }
    return T;
}

// // 解析雅可比矩阵：通过输入当前关节角得到
// MatrixXd computeJacobian(const vector<double>& q)
// {
//     vector<Matrix4d> T_list;
//     Matrix4d T = Matrix4d::Identity();
//     T_list.push_back(T);

//     for (int i = 0; i < 6; ++i)
//     {
//         T *= mdhTransform(a[i], alpha[i], d[i], q[i]);
//         T_list.push_back(T);
//     }

//     Vector3d pe = T_list[6].block<3,1>(0,3);
//     MatrixXd J(6,6);

//     for (int i = 0; i < 6; ++i)
//     {
//         Vector3d zi = T_list[i].block<3,1>(0,2);
//         Vector3d pi = T_list[i].block<3,1>(0,3);
//         Vector3d Ji_pos = zi.cross(pe - pi);
//         Vector3d Ji_ori = zi;

//         J.block<3,1>(0,i) = Ji_pos;
//         J.block<3,1>(3,i) = Ji_ori;
//     }
//     return J;
// }

// 数组雅可比
MatrixXd computeNumericJacobian(const vector<double>& q)
{
    const double eps = 1e-6;

    Matrix4d T0 = forwardKinematics(q);
    Vector3d p0 = T0.block<3,1>(0,3);
    Matrix3d R0 = T0.block<3,3>(0,0);

    MatrixXd J(6,6);
    for (int i = 0; i < 6; ++i)
    {
        vector<double> q_eps = q;
        q_eps[i] += eps;

        Matrix4d T1 = forwardKinematics(q_eps);
        Vector3d p1 = T1.block<3,1>(0,3);
        Matrix3d R1 = T1.block<3,3>(0,0);

        // 线速度部分
        Vector3d dp = (p1 - p0) / eps;

        // rotation vector 速度
        Matrix3d dR = R0.transpose() * R1;
        Vector3d dr = so3Log(dR) / eps;

        J.block<3,1>(0,i) = dp;
        J.block<3,1>(3,i) = dr;
    }
    return J;
}



// 关节状态回调
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    if(msg->position.size()!=6) return;

    current_joint.position=msg->position;
    q_ready=true;
    // cout << "打印 forwardKinematics(q):" << std::endl << forwardKinematics(current_joint.position) << std::endl;
}

// // 调试工具函数：
// Vector3d rotationMatrixToVector(const Matrix3d& R)
// {
//     AngleAxisd aa(R);
//     return aa.angle() * aa.axis();
// }
// MatrixXd computeNumericalJacobian(const vector<double>& q)
// {
//     const double eps = 1e-6;
//     MatrixXd Jnum(6,6);

//     Matrix4d T0 = forwardKinematics(q);
//     Vector3d p0 = T0.block<3,1>(0,3);
//     Vector3d r0 = rotationMatrixToVector(T0.block<3,3>(0,0));

//     for (int i = 0; i < 6; ++i)
//     {
//         vector<double> q_eps = q;
//         q_eps[i] += eps;

//         Matrix4d Ti = forwardKinematics(q_eps);
//         Vector3d pi = Ti.block<3,1>(0,3);
//         Vector3d ri = rotationMatrixToVector(Ti.block<3,3>(0,0));

//         // 数值微分
//         Jnum.block<3,1>(0,i) = (pi - p0) / eps;
//         Jnum.block<3,1>(3,i) = (ri - r0) / eps;
//     }
//     return Jnum;
// }


// 笛卡尔速度回调，并发布关节速度控制命令
void cartVelCallback(const robot_set::TCPState::ConstPtr &msg)
{
    if (!q_ready) return;
    if (msg->velocity.size() != 6) return;

    VectorXd vc(6);
    for (int i = 0; i < 6; ++i)
        vc(i) = msg->velocity[i];

    // MatrixXd J = computeJacobian(current_joint.position);
    MatrixXd J = computeNumericJacobian(current_joint.position);

    // SVD + 自适应阻尼
    JacobiSVD<MatrixXd> svd(J, ComputeThinU | ComputeThinV);
    double sigma_min = svd.singularValues().minCoeff();

    double lambda = 0.0;
    if (sigma_min < 0.05)
        lambda = 0.1 * (0.05 - sigma_min);

    MatrixXd I6 = MatrixXd::Identity(6,6);
    MatrixXd Jplus =
        J.transpose() * (J * J.transpose() + lambda*lambda * I6).inverse();

    VectorXd qdot = Jplus * vc;

    // 整体关节速度缩放
    const double MAX_QDOT[6] = {2.0,2.0,2.0,3.0,3.0,3.0};
    double scale = 1.0;
    for (int i = 0; i < 6; ++i)
        if (std::abs(qdot(i)) > 1e-6)
            scale = std::min(scale, MAX_QDOT[i] / std::abs(qdot(i)));

    if (scale < 1.0)
        qdot *= scale;

    sensor_msgs::JointState out;
    out.velocity.resize(6);
    for (int i = 0; i < 6; ++i)
        out.velocity[i] = qdot(i);

    out.header.stamp = ros::Time::now();
    pub.publish(out);

    cout << "当前各关节速度为：" << "[" << out.velocity[0] << "," << out.velocity[1] << "," << out.velocity[2] << "," 
        << out.velocity[3] << "," << out.velocity[4] << "," << out.velocity[5] << "]" << endl;
    
    // 调试输出
    // MatrixXd J_ana = computeJacobian(current_joint.position);
    MatrixXd J_num = computeNumericJacobian(current_joint.position);

    // cout << "J analytic:\n" << J_ana << endl;
    cout << "J numeric:\n" << J_num << endl;
    // cout << "diff:\n" << J_ana - J_num << endl;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_velocity_solver");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::JointState>("/joint_vel", 1);
    ros::Subscriber sub_vel = nh.subscribe("/cartesian_vel", 1, cartVelCallback);
    ros::Subscriber sub_joint = nh.subscribe("/joint_states", 1, jointStateCallback);
    // // 
    // ros::Rate rate(100);
    // while(ros::ok()){
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // cout << forwardKinematics({0.1,0.2,0.3,0.4,0.5,0.6})<<endl;
    ros::spin();
    return 0;
}
