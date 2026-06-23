
from paddleocr import PaddleOCR
import time
import os

# --- 这里开启 GPU ---
print("正在初始化 GPU 版本的 PaddleOCR...")
# use_gpu=True : 开启显卡加速
# use_angle_cls=True : 自动纠正倾斜的钢印
# lang='en' : 英文/数字模式
ocr = PaddleOCR(use_angle_cls=True, lang='en', use_gpu=True, show_log=False)

img_path = '/home/barry/workspace/ws_moveit/dataset/train/images/微信图片_20251212102851_459_41.jpg' # 请确保这里有一张你的测试图

if not os.path.exists(img_path):
    print(f"找不到图片: {img_path}，请放一张图来测试")
else:
    # 预热一次 (模型加载到显卡需要时间)
    ocr.ocr(img_path, cls=True) 
    print("模型预热完成，开始识别...")

    start_time = time.time()
    result = ocr.ocr(img_path, cls=True)
    end_time = time.time()

    print(f"耗时: {(end_time - start_time)*1000:.2f} ms")

    if result[0]:
        for idx, line in enumerate(result[0]):
            text = line[1][0]
            score = line[1][1]
            box = line[0]
            print(f"[{idx}] 内容: {text} | 置信度: {score:.4f} | 坐标: {box}")
            
            # 计算中心点，这正是你机械臂需要的坐标
            cx = (box[0][0] + box[2][0]) / 2
            cy = (box[0][1] + box[2][1]) / 2
            print(f"    -> 引导坐标 (u,v): ({cx:.1f}, {cy:.1f})")
    else:
        print("未检测到内容，请检查光照或图片清晰度。")