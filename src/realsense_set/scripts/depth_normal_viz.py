#!/usr/bin/env python3
"""
基于 PCA 的深度图法向量估计 + RViz 箭头可视化节点。

功能：
  订阅深度图像和相机内参，将图像划分为均匀网格，
  对每个网格单元的邻域点云做 PCA（协方差矩阵 SVD），
  取最小奇异值对应的特征向量作为法向量，
  最后以 MarkerArray（ARROW 类型）发布到 RViz 中显示。

话题约定（使用 realsense2_camera / rs_camera.launch 时）：
  /camera/depth/image_raw      -> sensor_msgs/Image (16UC1, 单位 mm)
  /camera/depth/camera_info    -> sensor_msgs/CameraInfo

发布话题：
  /normal_vectors              -> visualization_msgs/MarkerArray
"""

import sys
import numpy as np

import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class DepthNormalViz:
    """深度图 PCA 法向量估计与可视化节点。"""

    def __init__(self):
        # ================================================================
        #  ROS 参数（支持通过 launch 文件或命令行动态配置）
        # ================================================================
        self.grid_rows = rospy.get_param("~grid_rows", 40)           # 行方向网格数
        self.grid_cols = rospy.get_param("~grid_cols", 30)           # 列方向网格数
        self.window_size = rospy.get_param("~window_size", 7)        # PCA 邻域窗口大小（像素）
        self.arrow_length = rospy.get_param("~arrow_length", 0.03)   # 箭头长度（米）
        self.max_depth = rospy.get_param("~max_depth", 3.0)          # 最大有效深度（米）
        self.publish_rate = rospy.get_param("~publish_rate", 15.0)   # 发布频率（Hz）
        self.shaft_diam = rospy.get_param("~shaft_diameter", 0.002)  # 箭头杆直径
        self.head_diam = rospy.get_param("~head_diameter", 0.005)    # 箭头头部直径
        self.head_len = rospy.get_param("~head_length", 0.01)        # 箭头头部长度
        self.min_patch_points = rospy.get_param("~min_patch_points", 6)  # 邻域最少有效点数
        self.sor_k = rospy.get_param("~sor_k", 6)                      # SOR 近邻数
        self.sor_stddev = rospy.get_param("~sor_stddev", 1.0)          # SOR 标准差倍数阈值
        self.smooth_radius = rospy.get_param("~smooth_radius", 1)       # 法向量空间平滑邻域半径（1=3×3）
        self.smooth_sharpness = rospy.get_param("~smooth_sharpness", 8.0)  # 角度权重锐度，0=均匀平均

        # ================================================================
        #  相机内参（由 camera_info 回调填充）
        # ================================================================
        self.fx = 0.0               # X 方向焦距（像素）
        self.fy = 0.0               # Y 方向焦距（像素）
        self.cx = 0.0               # 主点 X 坐标（像素）
        self.cy = 0.0               # 主点 Y 坐标（像素）
        self.img_width = 0          # 图像宽度
        self.img_height = 0         # 图像高度
        self.camera_info_received = False  # 是否已收到内参

        # ================================================================
        #  cv_bridge：ROS Image <-> OpenCV/numpy 转换
        # ================================================================
        self.bridge = CvBridge()

        # ================================================================
        #  深度图缓存（回调中更新，定时器中处理）
        #  存储 float32 单位米，无效值为 0.0
        # ================================================================
        self.depth_lock = None              # 预留的线程锁（当前单线程定时器不需要）
        self.depth_img = None               # H×W float32 深度图（米）
        self.depth_timestamp = None         # 最新深度图的时间戳

        # ================================================================
        #  发布器：法向量可视化箭头
        # ================================================================
        self.marker_pub = rospy.Publisher(
            "/normal_vectors", MarkerArray, queue_size=10
        )

        # ================================================================
        #  订阅器：深度图像 + 相机内参
        # ================================================================
        rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.depth_callback,
            queue_size=1, buff_size=2**24
        )
        self.camera_info_sub = rospy.Subscriber(
            "/camera/depth/camera_info", CameraInfo, self.camera_info_callback,
            queue_size=1
        )

        # ================================================================
        #  定时器：按固定频率处理深度图并发布法向量
        # ================================================================
        self.proc_timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.process_and_publish
        )

        rospy.loginfo("depth_normal_viz: 节点已启动，等待 camera_info 和深度图...")

    # ------------------------------------------------------------------
    #  相机内参回调：收到第一条消息后提取 K 矩阵并取消订阅
    # ------------------------------------------------------------------
    def camera_info_callback(self, msg):
        """解析 CameraInfo 消息，提取内参矩阵 K 和图像尺寸。"""
        if self.camera_info_received:
            return
        # K 为 3×3 行优先矩阵：[fx, 0, cx, 0, fy, cy, 0, 0, 1]
        K = msg.K
        self.fx = K[0]
        self.fy = K[4]
        self.cx = K[2]
        self.cy = K[5]
        self.img_width = msg.width
        self.img_height = msg.height
        self.camera_info_received = True
        rospy.loginfo(
            "depth_normal_viz: 内参 K=[%.1f, %.1f, %.1f, %.1f] 分辨率=%dx%d",
            self.fx, self.fy, self.cx, self.cy,
            self.img_width, self.img_height
        )
        # 内参只需接收一次，之后取消订阅减少开销
        self.camera_info_sub.unregister()

    # ------------------------------------------------------------------
    #  深度图回调：将 16UC1（mm）转换为 float32（m）并缓存
    # ------------------------------------------------------------------
    def depth_callback(self, msg):
        """接收深度图像，转换为 float32 米制并过滤无效值。"""
        if not self.camera_info_received:
            return
        try:
            # 16UC1 (毫米) → float32 (米)
            img_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            depth = img_mm.astype(np.float32) * 0.001
            # 过滤无效深度：零值、负值、超出最大深度
            depth[(depth <= 0.0) | (depth > self.max_depth)] = 0.0
            self.depth_img = depth
            self.depth_timestamp = msg.header.stamp
        except Exception as e:
            rospy.logerr_throttle(5.0, "depth_normal_viz: cv_bridge 转换错误: %s", str(e))

    # ------------------------------------------------------------------
    #  SOR（Statistical Outlier Removal）离群点过滤
    #  对每个点计算到其 k 个最近邻的平均距离，移除平均距离偏离整体
    #  均值超过 stddev_mult 倍标准差的离群点。
    # ------------------------------------------------------------------
    @staticmethod
    def _sor_filter(pts, k=6, stddev_mult=1.0):
        """
        统计离群点移除。

        参数：
          pts        : (N, 3) 输入点云
          k          : 近邻数量
          stddev_mult: 标准差倍数阈值，越小越激进

        返回：
          filtered   : (M, 3) 去噪后的点云，M <= N
          mask       : (N,) bool 数组，True 表示保留
        """
        N = pts.shape[0]
        if N <= k:
            return pts, np.ones(N, dtype=bool)

        # 计算 N×N 欧氏距离矩阵的上三角
        diff = pts[:, None, :] - pts[None, :, :]           # (N, N, 3)
        dist = np.sqrt(np.sum(diff * diff, axis=2))         # (N, N)

        # 对每个点取距离最小的 k+1 个邻居（第 0 个是自己，距离=0，跳过）
        # 用 partition 避免全排序，取 k+1 个最小距离
        mean_dists = np.empty(N)
        for i in range(N):
            # partition: 最小的 k+1 个放到前面
            row = dist[i]
            partitioned = np.partition(row, k)
            # 取距离最小的 k+1 个（包含自己），去掉自己（距离 0）
            nearest = partitioned[:k + 1]
            # 排除自己：距离为 0 的那个
            nearest = nearest[nearest > 1e-12]
            if len(nearest) == 0:
                mean_dists[i] = 0.0
            else:
                mean_dists[i] = nearest.mean()

        # 全局统计
        global_mean = mean_dists.mean()
        global_std = mean_dists.std()
        threshold = global_mean + stddev_mult * global_std

        mask = mean_dists <= threshold
        # 如果过滤后点数太少，返回原始点云
        if mask.sum() < 3:
            return pts, np.ones(N, dtype=bool)

        return pts[mask], mask

    # ------------------------------------------------------------------
    #  定时器回调：网格化 → 反投影 → SOR → PCA → 空间平滑 → 发布
    # ------------------------------------------------------------------
    def process_and_publish(self, event):
        """
        两遍处理流程：
          第 1 遍 — 遍历所有网格单元，计算原始法向量和 3D 起点，存入二维数组
          第 2 遍 — 对每个网格取空间邻域，角度加权平均法向量，平滑后发布
        """
        if self.depth_img is None or not self.camera_info_received:
            return

        depth = self.depth_img
        H, W = depth.shape
        if H == 0 or W == 0:
            return

        R = self.grid_rows
        C = self.grid_cols
        half_win = self.window_size // 2

        # 预计算网格单元尺寸
        cell_h = H / float(R)
        cell_w = W / float(C)

        # ================================================================
        #  第 1 遍：计算所有网格的原始法向量和 3D 起点
        # ================================================================
        # normals_2d[r][c] = np.array([nx, ny, nz]) 或 None（无效）
        # origins_2d[r][c] = np.array([ox, oy, oz]) 或 None
        normals_2d = [[None] * C for _ in range(R)]
        origins_2d = [[None] * C for _ in range(R)]

        for r in range(R):
            for c in range(C):
                # ---- 网格中心像素 ----
                uc = int((c + 0.5) * cell_w)
                vc = int((r + 0.5) * cell_h)
                uc = max(0, min(W - 1, uc))
                vc = max(0, min(H - 1, vc))

                center_depth = depth[vc, uc]
                if center_depth <= 0.0:
                    continue

                # ---- 邻域深度块 ----
                r0 = max(0, vc - half_win)
                r1 = min(H, vc + half_win + 1)
                c0 = max(0, uc - half_win)
                c1 = min(W, uc + half_win + 1)

                patch = depth[r0:r1, c0:c1]
                valid_mask = patch > 0.0
                n_valid = np.count_nonzero(valid_mask)
                if n_valid < self.min_patch_points:
                    continue

                # ---- 反投影到 3D ----
                cols, rows = np.meshgrid(np.arange(c0, c1), np.arange(r0, r1))
                u = cols[valid_mask].astype(np.float64)
                v = rows[valid_mask].astype(np.float64)
                d = patch[valid_mask].astype(np.float64)

                x = (u - self.cx) * d / self.fx
                y = (v - self.cy) * d / self.fy
                z = d
                pts = np.stack([x, y, z], axis=1)

                # ---- SOR 去噪 ----
                pts, _ = self._sor_filter(pts, k=self.sor_k, stddev_mult=self.sor_stddev)
                n_valid = pts.shape[0]
                if n_valid < self.min_patch_points:
                    continue

                # ---- PCA 法向量 ----
                mu = pts.mean(axis=0)
                centered = pts - mu
                Cmat = centered.T @ centered / float(n_valid - 1)

                try:
                    U, S, Vt = np.linalg.svd(Cmat, full_matrices=False)
                except np.linalg.LinAlgError:
                    continue

                normal = Vt[-1].copy()
                if normal[2] > 0.0:
                    normal = -normal

                # ---- 3D 起点 ----
                ox = (uc - self.cx) * center_depth / self.fx
                oy = (vc - self.cy) * center_depth / self.fy
                oz = center_depth

                normals_2d[r][c] = normal
                origins_2d[r][c] = np.array([ox, oy, oz])

        # ================================================================
        #  第 2 遍：空间邻域加权平滑
        # ================================================================
        sr = self.smooth_radius
        sharp = self.smooth_sharpness

        markers = MarkerArray()
        # 每帧先清除上一帧全部箭头
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        delete_all.ns = "normals"
        markers.markers.append(delete_all)

        marker_id = 0

        for r in range(R):
            for c in range(C):
                normal = normals_2d[r][c]
                origin = origins_2d[r][c]
                if normal is None or origin is None:
                    continue

                # ---- 收集邻域法向量，角度加权平均 ----
                n_smooth = np.zeros(3)
                total_w = 0.0

                for dr in range(-sr, sr + 1):
                    for dc in range(-sr, sr + 1):
                        nr, nc = r + dr, c + dc
                        if nr < 0 or nr >= R or nc < 0 or nc >= C:
                            continue
                        nb = normals_2d[nr][nc]
                        if nb is None:
                            continue
                        if sharp > 0:
                            w = max(0.0, float(np.dot(normal, nb))) ** sharp
                        else:
                            w = 1.0
                        n_smooth += w * nb
                        total_w += w

                if total_w > 0.0:
                    n_smooth /= total_w
                    n_smooth /= np.linalg.norm(n_smooth)
                else:
                    n_smooth = normal

                # ---- 构造 ARROW Marker ----
                tip = origin + n_smooth * self.arrow_length

                marker = Marker()
                marker.header.frame_id = "camera_color_optical_frame"
                marker.header.stamp = self.depth_timestamp
                marker.ns = "normals"
                marker.id = marker_id
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.points = [
                    Point(x=origin[0], y=origin[1], z=origin[2]),
                    Point(x=tip[0],    y=tip[1],    z=tip[2]),
                ]
                marker.scale.x = self.shaft_diam
                marker.scale.y = self.head_diam
                marker.scale.z = self.head_len
                marker.color.r = max(0.0, min(1.0, abs(n_smooth[0])))
                marker.color.g = max(0.0, min(1.0, abs(n_smooth[1])))
                marker.color.b = max(0.0, min(1.0, abs(n_smooth[2])))
                marker.color.a = 0.85
                marker.lifetime = rospy.Duration(0)

                markers.markers.append(marker)
                marker_id += 1

        self.marker_pub.publish(markers)


# ======================================================================
#  主函数
# ======================================================================
def main():
    """初始化节点并进入 ROS 事件循环。"""
    rospy.init_node("depth_normal_viz", anonymous=True)
    node = DepthNormalViz()
    rospy.spin()


if __name__ == "__main__":
    main()
