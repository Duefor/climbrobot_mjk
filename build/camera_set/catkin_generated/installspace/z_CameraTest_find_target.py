# 识别到数字钢印并获得其相对于相机的位姿

import torch
from ultralytics import YOLO
import cv2
import numpy as np
import pyrealsense2 as rs

def rotationMatrixToEulerAngles(R):
    # 输入: 3x3旋转矩阵
    sy = np.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])

    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0

    return np.degrees(np.array([x, y, z]))  # 转成角度制


# 加载模型
model = YOLO('/home/duefor/climbrobot_mjk/runs/pose/digit_yolov8n_23/weights/best.pt')

# 启动realsense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.accel)
config.enable_stream(rs.stream.gyro)
profile = pipeline.start(config)

# 相机内参
color_intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
fx, fy = color_intrinsics.fx, color_intrinsics.fy
cx, cy = color_intrinsics.ppx, color_intrinsics.ppy
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]], dtype=np.float32)
dist_coeffs = np.zeros((5,))  # 假设无畸变

# 定义贴纸物体模型尺寸（单位：米）
sticker_size = 0.05  # 5厘米
object_points_world = np.array([
    [0, 0, 0],                          #左上角
    [sticker_size, 0, 0],               #右上角
    [sticker_size, sticker_size, 0],    #右下角
    [0, sticker_size, 0]                #左下角  与训练中关键点标注顺序一致
], dtype=np.float32)

# 初始化对齐器
align_to = rs.stream.color
align = rs.align(align_to)


# 主循环
while True:
    frames = pipeline.wait_for_frames()
    # 对齐深度图 → 彩色图坐标系
    aligned_frames = align.process(frames)

    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    if not color_frame or not depth_frame:
        continue

    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    # 推理
    results = model.predict(source=color_image, show=False, conf=0.5, verbose=False)

    # 拷贝图像用于画图
    vis_img = color_image.copy()

    found_target = False

    for result in results:
        keypoints = result.keypoints.xy.cpu().numpy()  # (N,4,2)
        classes = result.boxes.cls.cpu().numpy() if result.boxes.cls is not None else []

        if keypoints.shape[0] == 0:
            continue

        found_target = True

        for kp_set, cls_id in zip(keypoints, classes):
            # 提取角点
            points_2d = []
            object_points_camera = []

            for (x, y) in kp_set:
                u, v = int(x), int(y)

                # 加边界保护
                u = np.clip(u, 0, depth_image.shape[1] - 1)
                v = np.clip(v, 0, depth_image.shape[0] - 1)

                depth = depth_image[v, u] * 0.001  # 转米
                if depth == 0:
                    continue  # 跳过无效深度
                X = (u - cx) * depth / fx
                Y = (v - cy) * depth / fy
                Z = depth
                points_2d.append([u, v])
                object_points_camera.append([X, Y, Z])

            if len(points_2d) != 4:
                continue  # 如果角点不完整，跳过

            points_2d = np.array(points_2d, dtype=np.float32)
            object_points_camera = np.array(object_points_camera, dtype=np.float32)

            # 用PnP恢复位姿
            success, rvec, tvec = cv2.solvePnP(object_points_world, points_2d, camera_matrix, dist_coeffs)

            if success:
                R, _ = cv2.Rodrigues(rvec)  # 得到旋转矩阵

                # 求欧拉角
                euler_angles = rotationMatrixToEulerAngles(R)  # (roll, pitch, yaw)

                # 在图像上画出检测框
                x_min = int(min(kp_set[:, 0]))
                x_max = int(max(kp_set[:, 0]))
                y_min = int(min(kp_set[:, 1]))
                y_max = int(max(kp_set[:, 1]))
                cv2.rectangle(vis_img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            

                # 打印位姿信息
                cx = int(np.mean(kp_set[:, 0]))
                cy = int(np.mean(kp_set[:, 1]))
                tvec_text = f"XYZ: ({tvec[0][0]:.2f}, {tvec[1][0]:.2f}, {tvec[2][0]:.2f}) m"
                euler_text = f"RPY: ({euler_angles[0]:.1f},{euler_angles[1]:.1f},{euler_angles[2]:.1f}) deg"
                cv2.putText(vis_img, tvec_text, (cx-50, cy-20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                cv2.putText(vis_img, f'Class {int(cls_id)}', (cx-50, cy-40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                cv2.putText(vis_img, euler_text, (cx-70, cy-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)


    if not found_target:
        cv2.putText(vis_img, 'No Target Detected', (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

    # 显示结果
    cv2.imshow('Digit Sticker Detection + Pose Estimation', vis_img)

    if cv2.waitKey(1) & 0xFF == 27:
        break

# 释放资源
pipeline.stop()
cv2.destroyAllWindows()
