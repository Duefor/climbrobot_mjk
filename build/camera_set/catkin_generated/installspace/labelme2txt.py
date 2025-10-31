# 将labelme标注的json文件转化为yolo的txt文件

import os
import json
import cv2

# 配置路径
labelme_json_dir = '/home/duefor/climbrobot_mjk/src/camera_set/dataset/labels_json/val'  # 保存 labelme 标注文件的文件夹
output_label_dir = '/home/duefor/climbrobot_mjk/src/camera_set/dataset/labels/val'  # 输出标注目录

# 创建输出目录（如果不存在）
os.makedirs(output_label_dir, exist_ok=True)

def convert_labelme_to_yolopose(json_path, label_output_dir):
    with open(json_path, 'r') as f:
        data = json.load(f)

    img_path = data['imagePath']  #json文件中的原图片位置
    img_full_path = os.path.join(os.path.dirname(json_path), img_path)

    # 读取图片，拿到尺寸
    img = cv2.imread(img_full_path)
    if img is None:
        print(f"无法读取图片 {img_full_path}")
        return

    img_height, img_width = img.shape[:2]


    # 处理每一个标注对象
    label_txt_path = os.path.join(label_output_dir, os.path.splitext(os.path.basename(img_path))[0] + '.txt')
    with open(label_txt_path, 'w') as f_out:
        for shape in data['shapes']:
            label = shape['label']
            points = shape['points']  # list of (x,y)

            if len(points) != 4:
                print(f"警告：{json_path} 中 {label} 关键点数量不是4，跳过！")
                continue

            # 提取四个角点
            x1, y1 = points[0]
            x2, y2 = points[1]
            x3, y3 = points[2]
            x4, y4 = points[3]

            # 计算检测框（外接矩形）
            x_coords = [x1, x2, x3, x4]
            y_coords = [y1, y2, y3, y4]

            xmin = min(x_coords)
            xmax = max(x_coords)
            ymin = min(y_coords)
            ymax = max(y_coords)

            cx = (xmin + xmax) / 2 / img_width
            cy = (ymin + ymax) / 2 / img_height
            w = (xmax - xmin) / img_width
            h = (ymax - ymin) / img_height

            # 四个关键点归一化 + 可见性
            x1_n, y1_n = x1 / img_width, y1 / img_height
            x2_n, y2_n = x2 / img_width, y2 / img_height
            x3_n, y3_n = x3 / img_width, y3 / img_height
            x4_n, y4_n = x4 / img_width, y4 / img_height

            v = 2  # 可见性 2：可见

            # 假设label是数字（0-9），直接转成int
            try:
                class_id = int(label)
            except:
                print(f"警告：无法将label {label} 转为数字，跳过！")
                continue

            # 写入一行（YOLO pose格式）
            line = f"{class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f} " \
                   f"{x1_n:.6f} {y1_n:.6f} {v} {x2_n:.6f} {y2_n:.6f} {v} " \
                   f"{x3_n:.6f} {y3_n:.6f} {v} {x4_n:.6f} {y4_n:.6f} {v}\n"
            f_out.write(line)

# 主程序
json_list = [f for f in os.listdir(labelme_json_dir) if f.endswith('.json')]

for idx, json_file in enumerate(json_list):
    json_path = os.path.join(labelme_json_dir, json_file)
    convert_labelme_to_yolopose(json_path, output_label_dir)
    print(f"[{idx+1}/{len(json_list)}] 处理完成: {json_file}")

print("\n 全部Labelme文件已成功转换为YOLOv8-Pose格式！")
