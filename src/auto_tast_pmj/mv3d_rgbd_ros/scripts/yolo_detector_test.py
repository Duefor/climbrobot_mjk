import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch


# --- 核心修复：禁用坏掉的 cuDNN 引擎 ---
torch.backends.cudnn.enabled = False
torch.backends.cudnn.benchmark = False
torch.backends.cudnn.deterministic = True

class YoloCharacterDetector:
    def __init__(self):
        # 1. 初始化模型路径（请修改为你的 train8/weights/best.pt 绝对路径）
        model_path = '/home/barry/workspace/ws_moveit/runs/detect/train_v8_refined/weights/best.pt'
        self.model = YOLO(model_path)
        
        # 2. 初始化 ROS 相关
        rospy.init_node('yolo_character_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # 订阅 RGB 图像话题
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        
        rospy.loginfo("YOLO 字符检测节点已启动，正在监听 /camera/rgb ...")

    def image_callback(self, data):
            try:
                # 转换图像
                frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
                
                # 推理
                results = self.model.predict(source=frame, conf=0.3, device=0, verbose=False)
                
                # 获取带方框的渲染图
                annotated_frame = results[0].plot() 
                
                # 遍历结果，画中心点并打印坐标
                for box in results[0].boxes:
                    # 获取中心点 xywh (x_center, y_center, width, height)
                    xywh = box.xywh[0].cpu().numpy()
                    cx, cy = int(xywh[0]), int(xywh[1])
                    
                    # 在渲染图上额外画一个红色的实心圆点表示中心
                    cv2.circle(annotated_frame, (cx, cy), 5, (0, 0, 255), -1)
                    
                    # 终端打印，方便调试
                    cls_id = int(box.cls[0])
                    print(f"目标: {self.model.names[cls_id]} | 中心坐标: ({cx}, {cy})")

                # 显示图像
                cv2.imshow("YOLO Detection + Centers", annotated_frame)
                cv2.waitKey(1)

            except Exception as e:
                rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    detector = YoloCharacterDetector()
    rospy.spin()
    cv2.destroyAllWindows()