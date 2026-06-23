#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
print("PYTHON EXEC:", sys.executable)
print("FIRST 5 sys.path:")
for p in sys.path[:5]:
    print("  ", p)
import sys
import os

# ============================================================
# 关键：借用系统 ROS 的 Python 库（和你师兄一模一样）
# ============================================================
ros_path = '/opt/ros/noetic/lib/python3/dist-packages'
sys_path = '/usr/lib/python3/dist-packages'

if os.path.exists(ros_path) and ros_path not in sys.path:
    sys.path.append(ros_path)

if os.path.exists(sys_path) and sys_path not in sys.path:
    sys.path.append(sys_path)
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from paddleocr import PaddleOCR


class CameraOCRNode:
    def __init__(self):
        rospy.init_node("camera_ocr_node", anonymous=False)

        # ===================== 参数（直接写死） =====================
        self.rgb_image_topic = "/camera/rgb/image_raw"

        self.det_model_path = "/home/m/ws_moveit/PaddleOCR/new_model/det_steel_best_v2"
        self.rec_model_path = "/home/m/ws_moveit/PaddleOCR/new_model/rec_steel_best_v2"

        self.use_gpu = True
        self.visualize = True
        self.lang = "ch"    # ch / en

        # ===================== OCR 初始化 =====================
        rospy.loginfo("Loading PaddleOCR model...")
        self.ocr = PaddleOCR(
            det_model_dir=self.det_model_path,
            rec_model_dir=self.rec_model_path,
            use_angle_cls=False,
            use_gpu=self.use_gpu,
            lang=self.lang
        )
        rospy.loginfo("OCR model loaded")

        # ===================== ROS 接口 =====================
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(
            self.rgb_image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24
        )

        self.result_pub = rospy.Publisher(
            "/camera/ocr_result",
            String,
            queue_size=10
        )

    # ============================================================
    def image_callback(self, msg):
        # -------- ROS Image -> OpenCV --------
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # -------- OCR 推理 --------
        ocr_result = self.ocr.ocr(frame, cls=False)

        results_text = []

        if ocr_result is not None:
            for line in ocr_result:
                box = np.array(line[0], dtype=np.int32)
                text = line[1][0]
                score = line[1][1]

                results_text.append(f"{text}:{score:.2f}")

                if self.visualize:
                    cv2.polylines(frame, [box], True, (0, 255, 0), 2)
                    cv2.putText(
                        frame,
                        text,
                        (box[0][0], box[0][1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 0, 255),
                        2
                    )

        # -------- 发布识别结果 --------
        msg_out = String()
        msg_out.data = "; ".join(results_text)
        self.result_pub.publish(msg_out)

        # -------- 可视化 --------
        if self.visualize:
            cv2.imshow("camera_ocr", frame)
            cv2.waitKey(1)


if __name__ == "__main__":
    try:
        CameraOCRNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


