# 相机坐标系在第五关节坐标系下的位姿->变换矩阵
import numpy as np

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    # 归一化（防止数值误差）
    norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

    R = np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),         2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
    ])
    return R

def build_transform(x, y, z, qx, qy, qz, qw):
    R = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    t = np.array([x, y, z]).reshape(3, 1)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3:] = t
    return T

# ===== 你的数据 =====
x = -0.014353595893407988
y = 0.033661869271357885
z = 0.15485001375043025

qx = -0.6873409242223177
qy = -0.025767056214063055
qz = 0.026484172721386225
qw = 0.725394445318225

T = build_transform(x, y, z, qx, qy, qz, qw)

np.set_printoptions(precision=6, suppress=True)
print("变换矩阵 T_Link5_to_camera =\n", T)