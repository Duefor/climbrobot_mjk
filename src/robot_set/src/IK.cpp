// #include <iostream>
// #include <Eigen/Dense>
// using namespace std;
// using namespace Eigen;

// //ur3数据,这是学长给我的数据，适用于我的仿真环境，大家可以根据自己的环境进行修改
// // const double d[6+1] = { 0,0.1519,0,0,0.11235,0.08535,0.0819 };//第0个不用
// // const double a[6] = { 0,0,-0.24365,-0.21325,0,0 };//a有0，没有6
// const double d[6+1] = { 0,0.1625,0,0,0.1475,0.0965,0.092 };//第0个不用
// const double a[6] = { 0,0,-0.427,-0.3905,0,0 };//a有0，没有6
// const double alpha[6] = { 0,90,0,0,90,-90 };//alpha有0，没有6
// double theta[8 + 1][6 + 1];//八组解，每组解六个角，第0个都不用

// //正运动学，用于检验
// void kinematics(double* theta_input)
// {
// 	Eigen::Matrix4d T[6 + 1];//为了和theta对应，0不用
// 	for (int i = 1; i <= 6; i++)
// 	{
// 		T[i](0, 0) = cos(theta_input[i]);
// 		T[i](0, 1) = -sin(theta_input[i]);
// 		T[i](0, 2) = 0;
// 		T[i](0, 3) = a[i - 1];
// 		T[i](1, 0) = sin(theta_input[i]) * cos(alpha[i - 1] / 180 * EIGEN_PI);
// 		T[i](1, 1) = cos(theta_input[i]) * cos(alpha[i - 1] / 180 * EIGEN_PI);
// 		T[i](1, 2) = -sin(alpha[i - 1] / 180 * EIGEN_PI);
// 		T[i](1, 3) = -sin(alpha[i - 1] / 180 * EIGEN_PI) * d[i];
// 		T[i](2, 0) = sin(theta_input[i]) * sin(alpha[i - 1] / 180 * EIGEN_PI);
// 		T[i](2, 1) = cos(theta_input[i]) * sin(alpha[i - 1] / 180 * EIGEN_PI);
// 		T[i](2, 2) = cos(alpha[i - 1] / 180 * EIGEN_PI);
// 		T[i](2, 3) = cos(alpha[i - 1] / 180 * EIGEN_PI) * d[i];
// 		T[i](3, 0) = 0;
// 		T[i](3, 1) = 0;
// 		T[i](3, 2) = 0;
// 		T[i](3, 3) = 1;
// 	}
// 	Eigen::Matrix4d T06 = T[1] * T[2] * T[3] * T[4] * T[5] * T[6];
// 	cout << "检验得：X=" << T06(0, 3) * 1000 << "    Y=" << T06(1, 3) * 1000 << "     Z=" << T06(2, 3) * 1000;
// }
// int main()
// {
// 	double x, y, z, RX, RY, RZ;
// 	printf("Please input x,y,z,RX,RY,RZ:\n");
// 	scanf("%lf%lf%lf%lf%lf%lf", &x, &y, &z, &RX, &RY, &RZ);
// 	//由于仿真软件的单位不同，这边进行转换
// 	x = x / 1000;
// 	y = y / 1000;
// 	z = z / 1000;

// 	//1.旋转向量转旋转矩阵，即求T06中的r
// 	Eigen::Vector3d v(RX, RY, RZ);
// 	double t_alpha = v.norm();//求模
// 	v.normalize();//标准化
// 	Eigen::AngleAxisd rv(t_alpha, v);//旋转向量
// 	Eigen::Matrix3d rm;
// 	rm = rv.matrix();
	
// 	//2.求解
// 	double A, B, C, D, E, F, G, M, N;//用大写字母替代常数

// 	//注意，由于数组下标从0开始的问题，矩阵第一行第一列的元素是(0,0)
// 	//theta1
// 	A = rm(0, 2) * d[6] - x;
// 	B = rm(1, 2) * d[6] - y;
// 	C = d[4];
// 	//第一个解，赋给一到四组
// 	theta[1][1] = atan2(B, A) - atan2(C, sqrt(A * A + B * B - C * C));
// 	theta[2][1] = theta[1][1];
// 	theta[3][1] = theta[1][1];
// 	theta[4][1] = theta[1][1];
// 	//第二个解，赋给五到八组
// 	theta[5][1] = atan2(B, A) - atan2(C, -sqrt(A * A + B * B - C * C));
// 	theta[6][1] = theta[5][1];
// 	theta[7][1] = theta[5][1];
// 	theta[8][1] = theta[5][1];

// 	//theta5
// 	//由theta[1][1]产生的第一个解，赋给一到二组
// 	A = sin(theta[1][1]) * rm(0, 2) - cos(theta[1][1]) * rm(1, 2);
// 	theta[1][5] = atan2(sqrt(1 - A * A), A);
// 	theta[2][5] = theta[1][5];
// 	//由theta[1][1]产生的第二个解，赋给三到四组
// 	theta[3][5] = atan2(-sqrt(1 - A * A), A);
// 	theta[4][5] = theta[3][5];
// 	//由theta[5][1]产生的第一个解，赋给五到六组
// 	A = sin(theta[5][1]) * rm(0, 2) - cos(theta[5][1]) * rm(1, 2);
// 	theta[5][5] = atan2(sqrt(1 - A * A), A);
// 	theta[6][5] = theta[5][5];
// 	//由theta[5][1]产生的第二个解，赋给七到八组
// 	theta[7][5] = atan2(-sqrt(1 - A * A), A);
// 	theta[8][5] = theta[7][5];

// 	//theta6
// 	for (int i = 1; i <= 8; i++)
// 	{
// 		A = (-sin(theta[i][1]) * rm(0, 1) + cos(theta[i][1]) * rm(1, 1)) / theta[i][5];
// 		B = (sin(theta[i][1]) * rm(0, 0) - cos(theta[i][1]) * rm(1, 0)) / theta[i][5];
// 		theta[i][6] = atan2(A, B);
// 	}

// 	//theta2、theta3、theta4
// 	for (int i = 1; i <= 8; i = i + 2)
// 	{
// 		//先算theta2+theta3+theta4
// 		double theta234[8 + 1];
// 		A = rm(2, 2) / sin(theta[i][5]);
// 		B = (cos(theta[i][1]) * rm(0, 2) + sin(theta[i][1]) * rm(1, 2)) / sin(theta[i][5]);
// 		theta234[i] = atan2(-A, -B) - EIGEN_PI;
// 		theta234[i + 1] = theta234[i];

// 		//消去theta2+theta3，计算theta2
// 		A = -cos(theta234[i]) * sin(theta[i][5]) * d[6] + sin(theta234[i]) * d[5];
// 		B = -sin(theta234[i]) * sin(theta[i][5]) * d[6] - cos(theta234[i]) * d[5];
// 		C = cos(theta[i][1]) * x + sin(theta[i][1]) * y;
// 		D = z - d[1];
// 		M = C - A;
// 		N = D - B;
// 		E = -2 * N * a[2];
// 		F = 2 * M * a[2];
// 		G = M * M + N * N + a[2] * a[2] - a[3] * a[3];
// 		theta[i][2] = atan2(F, E) - atan2(G, sqrt(E * E + F * F - G * G));
// 		theta[i + 1][2] = atan2(F, E) - atan2(G, -sqrt(E * E + F * F - G * G));

// 		//用theta2反求theta2+theta3
// 		double theta23[8 + 1];
// 		theta23[i] = atan2((N - sin(theta[i][2]) * a[2]) / a[3], (M - cos(theta[i][2]) * a[2]) / a[3]);
// 		theta23[i + 1] = atan2((N - sin(theta[i + 1][2]) * a[2]) / a[3], (M - cos(theta[i + 1][2]) * a[2]) / a[3]);

// 		//theta3
// 		theta[i][3] = theta23[i] - theta[i][2];
// 		theta[i + 1][3] = theta23[i + 1] - theta[i + 1][2];

// 		//theta4
// 		theta[i][4] = theta234[i] - theta23[i];
// 		theta[i + 1][4] = theta234[i + 1] - theta23[i + 1];
// 	}

// 	//输出并检验
// 	for (int i = 1; i <= 8; i++)
// 	{
// 		cout << "第" << i << "组解：" << endl;
// 		for (int j = 1; j <= 6; j++)
// 			cout << "theta" << j << "=" << theta[i][j] * 180 / EIGEN_PI << "  ";
// 		cout << endl;
// 		kinematics(theta[i]);
// 		cout << endl << endl;
// 	}
// }

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
using namespace std;
using namespace Eigen;

// dh表
const double d[7]   = {0,0.1625,0,0,0.1475,0.0965,0.092};
const double a[6]   = {0,0,-0.427,-0.3905,0,0};
const double alpha[6] = {0,90,0,0,90,-90};
double theta[9][7]; // 共八组解

static double clamp(double v, double lo, double hi){ return v < lo ? lo : (v > hi ? hi : v); }
static double norm_angle(double ang){ // 归一化到 (-pi,pi]
    while(ang <= -M_PI) ang += 2*M_PI;
    while(ang >  M_PI) ang -= 2*M_PI;
    return ang;
}

// 正运动学校验
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
    cout << "检验得：X=" << T06(0,3) << "    Y=" << T06(1,3) << "     Z=" << T06(2,3) << endl;
}

int main()
{
    double x,y,z, RX,RY,RZ;
    printf("Please input x[m],y[m],z[m],RX[rad],RY[rad],RZ[rad]:\n");
    if(scanf("%lf%lf%lf%lf%lf%lf",&x,&y,&z,&RX,&RY,&RZ)!=6) return -1;

    // 旋转向量 -> 旋转矩阵
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

    // 归一化输出（单位 rad）
    for(int i=1;i<=8;i++){
        for(int j=1;j<=6;j++) theta[i][j] = norm_angle(theta[i][j]);
        cout << "第" << i << "组解：" << endl;
        for(int j=1;j<=6;j++) cout << "theta" << j << "=" << theta[i][j] << " rad  ";
        cout << endl;
        kinematics(theta[i]);
        cout << endl;
    }
    return 0;
}
