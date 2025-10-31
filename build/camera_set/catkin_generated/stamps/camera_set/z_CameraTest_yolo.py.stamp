# 测试训练模型


# from ultralytics import YOLO
# import cv2


# # 直接加载你的模型
# model = YOLO('/home/duefor/climbrobot_mjk/runs/detect/digit_yolov8n/weights/best.pt')

# cap = cv2.VideoCapture(2)

# digit_names = ['1', '9', '8', '7', '6', '5', '4', '3', '2', '0']

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         break

#     results = model.predict(source=frame, show=False, conf=0.5)

#     for result in results:
#         boxes = result.boxes.xyxy.cpu().numpy()
#         classes = result.boxes.cls.cpu().numpy()
#         confs = result.boxes.conf.cpu().numpy()

#         for box, cls, conf in zip(boxes, classes, confs):
#             x1, y1, x2, y2 = box
#             label = f'{digit_names[int(cls)]} {conf:.2f}'  # 用自定义类别名
#             cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
#             cv2.putText(frame, label, (int(x1), int(y1) - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

#     cv2.imshow('YOLO Detect', frame)

#     if cv2.waitKey(1) & 0xFF == 27:
#         break

# cap.release()
# cv2.destroyAllWindows()

###########################################################################################

from ultralytics import YOLO
import cv2
import glob  # 用于批量读取图片

# 加载训练好的模型
model = YOLO('/home/duefor/climbrobot_mjk/runs/detect/digit_yolov8n/weights/best.pt')

# 指定要检测的图片路径，可以是一张或一批
# image_paths = glob.glob('/home/duefor/climbrobot_mjk/test_images/*.jpg')  # 批量读取jpg图片
# 或者如果只检测一张图片的话
image_paths = ['/home/duefor/climbrobot_mjk/src/camera_set/dataset/images/train/049.jpg']

digit_names = ['1', '9', '8', '7', '6', '5', '4', '3', '2', '0']

for img_path in image_paths:
    frame = cv2.imread(img_path)
    if frame is None:
        print(f"无法读取图片 {img_path}")
        continue

    scale = 0.4  # 缩放比例（比如0.5就是缩小一半）
    frame = cv2.resize(frame, (0, 0), fx=scale, fy=scale)

    results = model.predict(source=frame, show=False, conf=0.5)

    for result in results:
        boxes = result.boxes.xyxy.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()

        for box, cls, conf in zip(boxes, classes, confs):
            x1, y1, x2, y2 = box
            label = f'{digit_names[int(cls)]} {conf:.2f}'
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, label, (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # 显示检测结果
    cv2.imshow('YOLO Image Detect', frame)
    key = cv2.waitKey(0)  # 等待按键继续
    if key == 27:  # 按ESC键退出
        break

cv2.destroyAllWindows()
