#!/usr/bin/env python3
import rospy
import cv2
import os
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageCollector:
    def __init__(self):
        # 1. 设置保存路径
        self.save_dir = "/home/barry/workspace/ws_moveit/src/mv3d_rgbd_ros/images_cylindrical"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        # 2. 初始化 ROS 和 Bridge
        rospy.init_node('image_collector', anonymous=True)
        self.bridge = CvBridge()
        self.latest_frame = None
        self.count = 0
        
        # 3. 订阅 RGB 话题
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        
        print("="*40)
        print(f"数据采集节点已启动")
        print(f"图片将保存至: {self.save_dir}")
        print("操作说明: 在此终端按 [Enter] 键拍照，输入 [q] 退出")
        print("="*40)

    def image_callback(self, data):
        try:
            # 持续获取最新一帧
            self.latest_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(f"图像转换失败: {e}")

    def run(self):
        while not rospy.is_shutdown():
            # 等待用户输入
            user_input = input(f"等待拍摄第 {self.count + 1} 张... (Enter拍照): ")
            
            if user_input.lower() == 'q':
                print("停止采集。")
                break
                
            if self.latest_frame is not None:
                # 生成唯一文件名 (时间戳格式)
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f"img_{timestamp}_{self.count}.jpg"
                file_path = os.path.join(self.save_dir, filename)
                
                # 保存图片
                cv2.imwrite(file_path, self.latest_frame)
                self.count += 1
                print(f"✅ 已保存: {filename}")
                
                # 可选：显示一下刚拍到的图（停留0.5秒）
                cv2.imshow("Captured", self.latest_frame)
                cv2.waitKey(500)
            else:
                print("❌ 错误：尚未接收到相机话题，请检查相机是否启动！")

if __name__ == '__main__':
    collector = ImageCollector()
    try:
        collector.run()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()