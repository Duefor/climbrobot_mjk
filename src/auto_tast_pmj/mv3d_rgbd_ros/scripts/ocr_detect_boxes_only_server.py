#!/home/barry/workspace/ws_moveit/venv_ocr/bin/python3
# -*- coding: utf-8 -*-

import os
import sys
import threading
import time

ros_path = '/opt/ros/noetic/lib/python3/dist-packages'
sys_path = '/usr/lib/python3/dist-packages'

if os.path.exists(ros_path) and ros_path not in sys.path:
    sys.path.append(ros_path)

if os.path.exists(sys_path) and sys_path not in sys.path:
    sys.path.append(sys_path)

import cv2
import message_filters
import numpy as np
import rospy
from paddleocr import PaddleOCR
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as SensorImage

from mv3d_rgbd_ros.srv import DetectSteelStamp, DetectSteelStampResponse


class OcrDetectBoxesOnlyServer:
    """
    仅相机输入的 OCR 检测服务：
    - 输入：同步的 RGB + CameraInfo
    - 输出：保存仅画框图片（不标注文字和置信度）到 image4 文件夹
    - 不依赖机械臂、MoveIt、TF
    """

    def __init__(self):
        rospy.init_node('ocr_detect_boxes_only_server')

        self.service_name = rospy.get_param('~service_name', 'detect_ocr_boxes_only')
        self.rgb_topic = rospy.get_param('~rgb_topic', '/camera/rgb/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/rgb/camera_info')
        self.sync_queue_size = int(rospy.get_param('~sync_queue_size', 10))
        self.sync_slop = float(rospy.get_param('~sync_slop', 0.05))

        self.det_model_path = rospy.get_param(
            '~det_model_dir',
            '/home/barry/workspace/ws_moveit/PaddleOCR/inference/det_steel_1280x720/',
        )
        self.rec_model_path = rospy.get_param(
            '~rec_model_dir',
            '/home/barry/workspace/ws_moveit/PaddleOCR/inference/rec_single_char/',
        )

        self.ocr = PaddleOCR(
            use_angle_cls=True,
            lang='en',
            use_gpu=True,
            det_model_dir=self.det_model_path,
            rec_model_dir=self.rec_model_path,
            rec=True,
            det_algorithm='DB',
            det_db_thresh=0.2,
            det_db_box_thresh=0.2,
            det_db_unclip_ratio=1.6,
            show_log=False,
        )

        script_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_root = os.path.dirname(script_dir)
        self.save_dir = os.path.join(pkg_root, 'image4')
        os.makedirs(self.save_dir, exist_ok=True)

        self.latest_data = {
            'rgb': None,
            'info': None,
        }
        self.data_lock = threading.Lock()

        rgb_sub = message_filters.Subscriber(self.rgb_topic, SensorImage)
        info_sub = message_filters.Subscriber(self.camera_info_topic, CameraInfo)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, info_sub],
            self.sync_queue_size,
            self.sync_slop,
        )
        self.sync.registerCallback(self.sync_callback)

        self.srv = rospy.Service(self.service_name, DetectSteelStamp, self.handle_req)

        rospy.loginfo('OCR 仅画框服务已启动: %s', self.service_name)
        rospy.loginfo('订阅: rgb=%s camera_info=%s', self.rgb_topic, self.camera_info_topic)
        rospy.loginfo('结果保存目录: %s', self.save_dir)

    def sync_callback(self, rgb_msg, info_msg):
        with self.data_lock:
            self.latest_data['rgb'] = rgb_msg
            self.latest_data['info'] = info_msg

    def handle_req(self, _req):
        response = DetectSteelStampResponse()
        response.success = False

        with self.data_lock:
            rgb_msg = self.latest_data['rgb']
            info_msg = self.latest_data['info']

        if rgb_msg is None or info_msg is None:
            response.message = 'No synchronized RGB/CameraInfo received yet'
            rospy.logwarn(response.message)
            return response

        try:
            rgb_arr = self.decode_bgr8(rgb_msg)
        except Exception as exc:
            response.message = 'Image parse error: {}'.format(exc)
            rospy.logerr(response.message)
            return response

        ocr_results = self.ocr.ocr(rgb_arr, cls=False)
        vis_image = rgb_arr.copy()

        box_count = 0
        if ocr_results and ocr_results[0]:
            for line in ocr_results[0]:
                bbox = np.asarray(line[0], dtype=np.float32)
                box_int = np.round(bbox).astype(np.int32)
                cv2.polylines(vis_image, [box_int], isClosed=True, color=(0, 255, 0), thickness=2)
                box_count += 1

                # 不在图上标注文字和置信度；仅返回文本结果供调试查看
                text = line[1][0]
                response.texts.append(text)

        save_path = self.save_result_image(vis_image)

        response.success = True
        response.message = 'Saved {} boxes to {}'.format(box_count, save_path)
        return response

    def decode_bgr8(self, msg):
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        row_stride = int(msg.step)
        expected = int(msg.width) * 3
        image = raw.reshape(int(msg.height), row_stride)[:, :expected]
        return image.reshape(int(msg.height), int(msg.width), 3).copy()

    def save_result_image(self, image):
        filename = 'ocr_boxes_{}.jpg'.format(int(time.time() * 1000))
        full_path = os.path.join(self.save_dir, filename)
        cv2.imwrite(full_path, image)
        return full_path


if __name__ == '__main__':
    try:
        node = OcrDetectBoxesOnlyServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
