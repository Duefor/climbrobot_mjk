# yolo训练模型

from ultralytics import YOLO

# 加载一个预训练模型 (yolov8n, yolov8s, yolov8m, yolov8l, yolov8x)
model = YOLO('/home/duefor/climbrobot_mjk/src/camera_set/model/yolov8n-pose.pt')  # 此为关键点检测模型

# 开始训练
model.train(
    data='/home/duefor/climbrobot_mjk/src/camera_set/dataset/target.yaml',  # 数据集yaml文件
    epochs=100,            # 训练轮数
    imgsz=640,             # 输入图片尺寸
    batch=16,              # 每批次处理图片数量
    patience=20,           # 早停策略：20轮内不提升就停止
    device='cpu',              # 指定GPU，如果没有GPU可以改成 'cpu'
    workers=8,             # 读数据的线程数
    optimizer='SGD',       # 优化器（SGD, Adam, AdamW）
    name='digit_yolov8n_2'   # 保存目录名字 
)
