import cv2
import numpy as np
import os
import random
import gc  # 引入垃圾回收机制

def process_single_image(img, target_height=720):
    """
    输入原图，输出处理后的小尺寸、模拟工业相机效果图
    """
    # ================= 阶段 1: 极速瘦身 (内存优化核心) =================
    # 1. 获取原始尺寸
    h, w = img.shape[:2]
    
    # 2. 计算缩放比例 (目标高度 720px)
    # 4032x3024 -> 960x720
    scale = target_height / float(h)
    new_w = int(w * scale)
    
    # 3. 执行缩放 (使用 INTER_AREA 插值，缩小效果最好且去噪)
    # 这一步之后，图像数据量减少了约 16 倍！后续处理都在这个小图上进行，内存占用极低。
    resized_img = cv2.resize(img, (new_w, target_height), interpolation=cv2.INTER_AREA)
    
    # ================= 阶段 2: 工业相机模拟 (效果合成) =================
    
    # 1. 模拟传感器噪声 (Sensor Noise)
    # 生成与小图尺寸一致的噪声矩阵
    sigma = random.randint(5, 12)  # 噪点强度
    gauss = np.random.normal(0, sigma, resized_img.shape).astype('int16')
    # 加噪并截断到 0-255
    noisy_img = np.clip(resized_img.astype('int16') + gauss, 0, 255).astype('uint8')

    # 2. 模拟模糊 (Blur) - 模拟对焦不准或运动拖影
    if random.random() < 0.5:
        # 高斯模糊 (对焦不准)
        k_size = random.choice([3, 5])
        blurred_img = cv2.GaussianBlur(noisy_img, (k_size, k_size), 0)
    else:
        # 运动模糊 (传送带/机械臂移动)
        kernel_size = random.choice([3, 5])
        kernel = np.zeros((kernel_size, kernel_size))
        if random.random() < 0.5:
            kernel[int((kernel_size-1)/2), :] = np.ones(kernel_size) # 水平
        else:
            kernel[:, int((kernel_size-1)/2)] = np.ones(kernel_size) # 垂直
        kernel /= kernel_size
        blurred_img = cv2.filter2D(noisy_img, -1, kernel)

    # 3. 模拟光照不均匀 (Illumination)
    alpha = random.uniform(0.7, 1.1) # 工业相机对比度通常稍低
    beta = random.randint(-20, 20)   # 亮度波动
    adjusted = cv2.convertScaleAbs(blurred_img, alpha=alpha, beta=beta)

    # 4. 模拟色偏 (Color Cast) - 模拟暖光或冷光环境
    if random.random() < 0.3: 
        b_gain = random.uniform(0.9, 1.1)
        r_gain = random.uniform(0.9, 1.1)
        b, g, r = cv2.split(adjusted)
        b = cv2.multiply(b, b_gain) # 调整蓝色通道
        r = cv2.multiply(r, r_gain) # 调整红色通道
        merged = cv2.merge([b, g, r])
        adjusted = np.clip(merged, 0, 255).astype('uint8')

    return adjusted

def batch_process_optimization(input_folder, output_folder, target_h=720):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    files = [f for f in os.listdir(input_folder) if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))]
    total = len(files)
    print(f"🚀 启动高效处理管线: 目标 {total} 张图片")
    print(f"📉 目标分辨率高度: {target_h}p (自动保持宽高比)")
    print(f"💾 输出画质: JPEG Quality 85 (视觉无损，体积更小)")

    success_count = 0
    
    for i, f in enumerate(files):
        img_path = os.path.join(input_folder, f)
        
        try:
            # 1. 读取大图
            # 注意：这是内存占用的峰值时刻，约需 30-50MB RAM
            img = cv2.imread(img_path)
            
            if img is None:
                print(f"⚠️ 跳过损坏文件: {f}")
                continue

            # 2. 处理 (缩放 + 特效)
            processed_img = process_single_image(img, target_height=target_h)

            # 3. 立即释放大图内存 (关键步骤！)
            del img 
            
            # 4. 保存结果
            # 使用 JPEG 格式，质量设为 85 (体积小，画质好)
            out_path = os.path.join(output_folder, f)
            # 如果原图是 png，强制改为 jpg 以节省空间
            if out_path.endswith(".png"):
                out_path = out_path.replace(".png", ".jpg")
                
            cv2.imwrite(out_path, processed_img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            
            # 5. 手动触发垃圾回收，防止内存泄漏
            if i % 10 == 0:
                gc.collect()

            success_count += 1
            if success_count % 10 == 0:
                print(f"✅ 已完成 {success_count}/{total} ...")
                
        except Exception as e:
            print(f"❌ 处理 {f} 失败: {e}")

    print(f"\n🎉 全部完成！")
    print(f"📁 结果保存在: {os.path.abspath(output_folder)}")
    print("💡 提示：请使用 PPOCRLabel 打开该文件夹进行标注。")

if __name__ == "__main__":
    # ================= 配置区域 =================
    # 把你手机拍的超大图直接放这里
    INPUT_DIR = "/home/barry/workspace/ws_moveit/src/mv3d_rgbd_ros/mobile_photos"       
    
    # 脚本会自动把图片缩小、处理、压缩后放这里
    OUTPUT_DIR = "/home/barry/workspace/ws_moveit/src/mv3d_rgbd_ros/training_dataset"   
    # ===========================================
    
    batch_process_optimization(INPUT_DIR, OUTPUT_DIR, target_h=720)