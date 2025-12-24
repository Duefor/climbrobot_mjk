#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using Eigen::Vector3d;
using std::cout;
using std::endl;

int main()
{

    // 机械臂 xyz 实际活动范围
    Vector3d Umin(-0.8, -0.8,  -0.0);
    Vector3d Umax( 0.8,  0.8,  0.8);

    // 手柄 xyz 实际活动范围
    Vector3d Pmin(-0.21, -0.08, -0.11);
    Vector3d Pmax( 0.21,  0.08,  0.18);

    // 手柄初始位置 O_P
    Vector3d Op(0.0,0.08,-0.065);


    Vector3d Ou;   // 机械臂初始位置（偏移量）
    Vector3d k;    // 映射比例

    for(int i = 0; i < 3; ++i)
    {
        // O_Ui = Uimin + |POi - Pimin| / (Pimax - Pimin) * (Uimax - Uimin)
        Ou[i] =
            Umin[i]
            + std::abs(Op[i] - Pmin[i])
              / (Pmax[i] - Pmin[i])
              * (Umax[i] - Umin[i]);

        // k_i = (Uimax - OUi) / (Pimax - OPi)
        k[i] =
            (Umax[i] - Ou[i])
            / (Pmax[i] - Op[i]);
    }


    cout << "========= 映射结果 =========" << endl;

    cout << "机械臂初始位置 O_U (偏移量):" << endl;
    cout << "  x: " << Ou[0] << endl;
    cout << "  y: " << Ou[1] << endl;
    cout << "  z: " << Ou[2] << endl;

    cout << endl;

    cout << "映射比例 k:" << endl;
    cout << "  kx: " << k[0] << endl;
    cout << "  ky: " << k[1] << endl;
    cout << "  kz: " << k[2] << endl;

    cout << "============================" << endl;

    return 0;
}