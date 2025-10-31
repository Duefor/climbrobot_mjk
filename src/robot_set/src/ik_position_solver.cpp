// 接收机械臂笛卡尔空间下的目标点并转换为关节空间下的目标点发送

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
// #include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <sensor_msgs/JointState.h>
#include <robot_set/TCPState.h>

using namespace std;
using namespace Eigen;

double freq = 10;

// dh表
const double d[6+1] = { 0,0.1625,0,0,0.1475,0.0965,0.092 };//第0个不用
const double a[6] = { 0,0,-0.427,-0.3905,0,0 };//a有0，没有6
const double alpha[6] = { 0,90,0,0,90,-90 };//alpha有0，没有6
double theta[8 + 1][6 + 1];//八组解，每组解六个角，第0个都不用

// 全局缓存位姿
robot_set::TCPState latest_pose;    // 最新笛卡尔空间位姿
bool pose_received = false; // 标记是否接收数据

// 解的选取
vector<double> prev_joints = {0.1, 0.2, 0.1, 0, 0, 0};   // 上一帧解，初始为初始位姿角

// 选取解
int selectClosestSolution(const vector<vector<double>>& solutions, const vector<double>& prev_joints){

    double min_dist = std::numeric_limits<double>::max();
    int best_index = 0;

    for (int i = 0; i < solutions.size(); i++) {
        double dist = 0.0;
        for (int j = 0; j < 6; j++) {
            double diff = solutions[i][j] - prev_joints[j];

            // 防止角度跳变（考虑周期性）
            while (diff > M_PI) diff -= 2 * M_PI;
            while (diff < -M_PI) diff += 2 * M_PI;

            dist += diff * diff;
        }
        if (dist < min_dist) {
            min_dist = dist;
            best_index = i;
        }
    }
    return best_index;
}


static double clamp(double v, double lo, double hi){
	return v < lo ? lo : (v > hi ? hi : v);
}
// 归一化到 (-pi,pi]
static double norm_angle(double ang){
    while(ang <= -M_PI) ang += 2*M_PI;
    while(ang >  M_PI) ang -= 2*M_PI;
    return ang;
}

//正运动学，用于检验
void kinematics(const double* th)
{
    Matrix4d T[7];
    for(int i=1;i<=6;i++){
        double th_i = th[i];
        double a_i1 = a[i-1];
        double alpha_rad = alpha[i-1] * M_PI / 180.0;
        T[i] << cos(th_i), -sin(th_i), 0, a_i1,
                sin(th_i)*cos(alpha_rad), cos(th_i)*cos(alpha_rad), -sin(alpha_rad), -sin(alpha_rad)*d[i],
                sin(th_i)*sin(alpha_rad), cos(th_i)*sin(alpha_rad),  cos(alpha_rad),  cos(alpha_rad)*d[i],
                0,0,0,1;
    }
    Matrix4d T06 = T[1]*T[2]*T[3]*T[4]*T[5]*T[6];
    cout << T06 << endl;
    cout << "检验得：X=" << T06(0,3) << "    Y=" << T06(1,3) << "     Z=" << T06(2,3) << endl;
}

// 求解ik
int solveIK(double x, double y, double z, double RX, double RY, double RZ, vector<vector<double>>& solutions)
{
    // 旋转向量转化为旋转矩阵
    Vector3d v(RX,RY,RZ);
    double t_alpha = v.norm();
    Matrix3d rm = Matrix3d::Identity();
    if(t_alpha > 1e-12){
        v /= t_alpha;	// 单位化
        AngleAxisd rv(t_alpha, v);
        rm = rv.matrix();
    }

    // theta1
    double A = rm(0,2)*d[6] - x;
    double B = rm(1,2)*d[6] - y;
    double C = d[4];
    double tmp = A*A + B*B - C*C;
    if(tmp < 0) tmp = 0;
    double s = sqrt(tmp);

    for(int k=1;k<=4;k++) theta[k][1] = atan2(B,A) - atan2(C, s);
    for(int k=5;k<=8;k++) theta[k][1] = atan2(B,A) - atan2(C,-s);

    // theta5
    for(int base : {1,5}){
        int idx1 = base;
        double th1group = theta[idx1][1];
        double Atmp = sin(th1group)*rm(0,2) - cos(th1group)*rm(1,2);
        double val = clamp(Atmp,-1.0,1.0);
        double th5_pos = atan2(sqrt(max(0.0,1-val*val)), val);
        double th5_neg = atan2(-sqrt(max(0.0,1-val*val)), val);

        if(base==1){
            theta[1][5]=th5_pos; theta[2][5]=th5_pos;
            theta[3][5]=th5_neg; theta[4][5]=th5_neg;
        }else{
            theta[5][5]=th5_pos; theta[6][5]=th5_pos;
            theta[7][5]=th5_neg; theta[8][5]=th5_neg;
        }
    }

    // theta6
    for(int i=1;i<=8;i++){
        double th5 = theta[i][5];
        if(fabs(sin(th5)) < 1e-8){
            theta[i][6] = 0.0;
        } else {
            double A6 = (-sin(theta[i][1])*rm(0,1) + cos(theta[i][1])*rm(1,1))/sin(th5);
            double B6 = ( sin(theta[i][1])*rm(0,0) - cos(theta[i][1])*rm(1,0))/sin(th5);
            theta[i][6] = atan2(A6,B6);
        }
    }

    // theta2,3,4
    for(int i=1;i<=8;i+=2){
        double th5 = theta[i][5];
        if(fabs(sin(th5)) < 1e-8){
            theta[i][2]=theta[i+1][2]=0;
            theta[i][3]=theta[i+1][3]=0;
            theta[i][4]=theta[i+1][4]=0;
            continue;
        }

        double theta234[9];
        double A23 = rm(2,2)/sin(th5);
        double B23 = (cos(theta[i][1])*rm(0,2) + sin(theta[i][1])*rm(1,2))/sin(th5);
        theta234[i]   = atan2(-A23,-B23) - M_PI;
        theta234[i+1] = theta234[i];

        double Acons = -cos(theta234[i])*sin(th5)*d[6] + sin(theta234[i])*d[5];
        double Bcons = -sin(theta234[i])*sin(th5)*d[6] - cos(theta234[i])*d[5];
        double Cc = cos(theta[i][1])*x + sin(theta[i][1])*y;
        double Dd = z - d[1];
        double M = Cc - Acons;
        double N = Dd - Bcons;

        double E = -2*N*a[2];
        double F =  2*M*a[2];
        double G = M*M + N*N + a[2]*a[2] - a[3]*a[3];

        double disc = E*E + F*F - G*G;
        if(disc < 0) disc = 0;
        double sdisc = sqrt(disc);

        theta[i][2]   = atan2(F,E) - atan2(G, sdisc);
        theta[i+1][2] = atan2(F,E) - atan2(G,-sdisc);

        double theta23[9];
        theta23[i]   = atan2((N - sin(theta[i][2])*a[2])/a[3],   (M - cos(theta[i][2])*a[2])/a[3]);
        theta23[i+1] = atan2((N - sin(theta[i+1][2])*a[2])/a[3], (M - cos(theta[i+1][2])*a[2])/a[3]);

        theta[i][3]   = theta23[i]   - theta[i][2];
        theta[i+1][3] = theta23[i+1] - theta[i+1][2];

        theta[i][4]   = theta234[i]   - theta23[i];
        theta[i+1][4] = theta234[i+1] - theta23[i+1];
    }
	// 保存到 solutions 中
    for (int i=1;i<=8;i++){
        vector<double> sol;
        for (int j=1;j<=6;j++)
            sol.push_back(theta[i][j]);
        solutions.push_back(sol);
    }
    return solutions.size();
}

void poseCallback(const robot_set::TCPState::ConstPtr& msg)
{
	if(msg->position.size() != 6) return;
    latest_pose = *msg;
    pose_received = true;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "ik_position_solver");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("/cartesian_pose", 10, poseCallback);
    // ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states_target", 10);
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);  // 用于测试仿真的话题名称

    ros::Rate loop_rate(freq); // 50 Hz 发布频率，但是注意该频率不会大于订阅频率

    sensor_msgs::JointState joint_msg;
    joint_msg.position.resize(6);
    joint_msg.name = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"}; // 与urdf中名称一致
    while (ros::ok()) {
        ros::spinOnce();
        if (pose_received) {
            double x = latest_pose.position[0];
            double y = latest_pose.position[1];
            double z = latest_pose.position[2];
            double RX = latest_pose.position[3];
            double RY = latest_pose.position[4];
            double RZ = latest_pose.position[5];

            vector<vector<double>> solutions;
            int num_solutions = solveIK(x,y,z,RX,RY,RZ,solutions);

            if (num_solutions > 0) {
                // 进行解的选取
                int best_idx = selectClosestSolution(solutions, prev_joints);

                for (int j=0;j<6;j++){
                    // 这里可以考虑采用低通滤波平滑输出
                    // joint_msg.data[j] = 0.9 * prev_joints[j] + 0.1 * solutions[best_idx][j];

                    joint_msg.position[j] = solutions[best_idx][j]; // 取最优解
                    prev_joints[j] = joint_msg.position[j]; // 记录上一组解
                }
                joint_msg.header.stamp = ros::Time::now();  // 时间戳
                joint_pub.publish(joint_msg);
                ROS_INFO_STREAM("发布转换后的关节角坐标: [" << joint_msg.position[0] << ", "<< joint_msg.position[1] << ", "<< joint_msg.position[2] << ", "
                	<< joint_msg.position[3] << ", "<< joint_msg.position[4] << ", "<< joint_msg.position[5] << "]");
                
                // // 检验
                // double abc[7] = {0, joint_msg.position[0], joint_msg.position[1], joint_msg.position[2], joint_msg.position[3], joint_msg.position[4], joint_msg.position[5]};
                // kinematics(abc);
            }
			pose_received = false;	//保证发布关节角坐标频率不会高于接收tcp坐标的频率
        }
        loop_rate.sleep();
    }
    return 0;
}