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

// 解析雅可比矩阵：通过输入当前关节角得到
MatrixXd computeJacobian(const vector<double>& q)
{
    vector<Matrix4d> T_list;
    Matrix4d T = Matrix4d::Identity();
    T_list.push_back(T);

    for (int i = 0; i < 6; ++i)
    {
        T *= mdhTransform(a[i], alpha[i], d[i], q[i]);
        T_list.push_back(T);
    }

    Vector3d pe = T_list[6].block<3,1>(0,3);
    MatrixXd J(6,6);

    for (int i = 0; i < 6; ++i)
    {
        Vector3d zi = T_list[i].block<3,1>(0,2);
        Vector3d pi = T_list[i].block<3,1>(0,3);
        Vector3d Ji_pos = zi.cross(pe - pi);
        Vector3d Ji_ori = zi;

        J.block<3,1>(0,i) = Ji_pos;
        J.block<3,1>(3,i) = Ji_ori;
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


// 笛卡尔速度回调，并发布关节速度控制命令
void cartVelCallback(const robot_set::TCPState::ConstPtr &msg)
{
    if(!q_ready) return;
    q_ready = false;
    if(msg->velocity.size()!=6) return;

    VectorXd vc(6);
    for(int i=0;i<6;++i) vc(i)=msg->velocity[i];

    MatrixXd J = computeJacobian(current_joint.position);
    // 阻尼系数，先用一个定值
    double lambda = 0.01;
    MatrixXd I6 = MatrixXd::Identity(6,6);
    // 阻尼最小二乘处理奇异点
    MatrixXd Jplus = J.transpose() * (J * J.transpose() + lambda*lambda * I6).inverse();
    VectorXd qdot = Jplus * vc;
    // VectorXd qdot = J.inverse() * vc;


    // 限幅
    // for(int i=0;i<6;++i)
    //     qdot(i) = std::max(std::min(qdot(i), 1.0), -1.0); // rad/s

    sensor_msgs::JointState out;
    out.velocity.resize(6);
    for(int i=0;i<6;++i) out.velocity[i]=qdot(i);
    out.name = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
    out.header.stamp = ros::Time::now();
    pub.publish(out);
    cout << "当前各关节速度为：" << "[" << out.velocity[0] << "," << out.velocity[1] << "," << out.velocity[2] << "," 
        << out.velocity[3] << "," << out.velocity[4] << "," << out.velocity[5] << "]" << endl;

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
