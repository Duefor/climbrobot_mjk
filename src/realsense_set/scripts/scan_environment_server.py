#!/usr/bin/env python3
"""
环境扫描动作服务器。

收到扫描请求时：
  1. 采集深度帧（可选平均 N 帧降噪）
  2. 网格化 → 反投影 → SOR 去噪 → PCA 法向量 → 空间平滑
  3. TF 查询相机→基座变换，将位置和法向量转换到 base_link 下
  4. 用法向量构造完整 6D 姿态（Z=法向量，X=基座X投影，Y=Z×X）
  5. 返回采样点列表（geometry_msgs/Pose[]）
  6. 同步发布 MarkerArray 供 RViz 可视化

动作话题：
  /scan_environment/goal, /scan_environment/result, /scan_environment/feedback
"""

import sys
import threading
import numpy as np

import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3

import tf.transformations as tr

from realsense_set.msg import ScanEnvironmentAction, ScanEnvironmentResult, ScanEnvironmentFeedback


# ======================================================================
#  姿态构造：法向量 → 6D 位姿
# ======================================================================
def normal_to_quaternion(normal, base_x=np.array([1.0, 0.0, 0.0])):
    """
    从单位法向量构造表示姿态的四元数。

    Z 轴 = 法向量（工具接近方向）
    X 轴 = base_x 在法向量正交平面上的投影（Gram-Schmidt）
    Y 轴 = Z × X（右手系）

    若法向量与 base_x 平行，改用 base_y=[0,1,0] 投影。
    """
    Z = normal / np.linalg.norm(normal)
    X = base_x - np.dot(base_x, Z) * Z          # Gram-Schmidt 投影
    if np.linalg.norm(X) < 1e-6:
        base_y = np.array([0.0, 1.0, 0.0])
        X = base_y - np.dot(base_y, Z) * Z
    X = X / np.linalg.norm(X)
    Y = np.cross(Z, X)
    Y = Y / np.linalg.norm(Y)

    R = np.column_stack([X, Y, Z])               # 3×3 旋转矩阵
    R_homo = np.eye(4)
    R_homo[:3, :3] = R
    q = tr.quaternion_from_matrix(R_homo)        # [x, y, z, w]
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


# ======================================================================
#  SOR 统计离群点过滤
# ======================================================================
def sor_filter(pts, k=6, stddev_mult=1.0):
    """
    统计离群点移除（Statistical Outlier Removal）。

    对每个点计算到其 k 个最近邻的平均距离，
    移除平均距离超过 (全局均值 + stddev_mult × 全局标准差) 的离群点。

    返回：(filtered_pts, keep_mask)
    """
    N = pts.shape[0]
    if N <= k:
        return pts, np.ones(N, dtype=bool)

    diff = pts[:, None, :] - pts[None, :, :]            # (N, N, 3)
    dist = np.sqrt(np.sum(diff * diff, axis=2))          # (N, N)

    mean_dists = np.empty(N)
    for i in range(N):
        row = dist[i]
        partitioned = np.partition(row, k)
        nearest = partitioned[:k + 1]
        nearest = nearest[nearest > 1e-12]
        mean_dists[i] = nearest.mean() if len(nearest) > 0 else 0.0

    global_mean = mean_dists.mean()
    global_std = mean_dists.std()
    threshold = global_mean + stddev_mult * global_std

    mask = mean_dists <= threshold
    if mask.sum() < 3:
        return pts, np.ones(N, dtype=bool)
    return pts[mask], mask


# ======================================================================
#  环境扫描动作服务器
# ======================================================================
class ScanEnvironmentServer:
    """环境扫描动作服务器。"""

    def __init__(self):
        # ================================================================
        #  ROS 参数
        # ================================================================
        self.grid_rows = rospy.get_param("~grid_rows", 40)
        self.grid_cols = rospy.get_param("~grid_cols", 30)
        self.window_size = rospy.get_param("~window_size", 7)
        self.arrow_length = rospy.get_param("~arrow_length", 0.03)
        self.max_depth = rospy.get_param("~max_depth", 3.0)
        self.shaft_diam = rospy.get_param("~shaft_diameter", 0.002)
        self.head_diam = rospy.get_param("~head_diameter", 0.005)
        self.head_len = rospy.get_param("~head_length", 0.01)
        self.min_patch_points = rospy.get_param("~min_patch_points", 6)
        self.sor_k = rospy.get_param("~sor_k", 6)
        self.sor_stddev = rospy.get_param("~sor_stddev", 1.0)
        self.smooth_radius = rospy.get_param("~smooth_radius", 1)
        self.smooth_sharpness = rospy.get_param("~smooth_sharpness", 8.0)
        self.scan_frame_average = rospy.get_param("~scan_frame_average", 3)
        self.camera_frame = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.target_frame = rospy.get_param("~target_frame", "base_link")

        # ================================================================
        #  相机内参（camera_info 回调填充）
        # ================================================================
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.img_width = 0
        self.img_height = 0
        self.camera_info_received = False

        # ================================================================
        #  cv_bridge
        # ================================================================
        self.bridge = CvBridge()

        # ================================================================
        #  最新深度图缓存（回调更新，扫描时读取）
        # ================================================================
        self.depth_lock = threading.Lock()
        self.depth_img = None               # H×W float32（米），无效值=0.0
        self.depth_timestamp = None

        # ================================================================
        #  TF2
        # ================================================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ================================================================
        #  发布器：RViz 可视化箭头
        # ================================================================
        self.marker_pub = rospy.Publisher(
            "/scan_environment/markers", MarkerArray, queue_size=1
        )

        # ================================================================
        #  订阅器
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
        #  动作服务器
        # ================================================================
        self.server = actionlib.SimpleActionServer(
            "scan_environment",
            ScanEnvironmentAction,
            execute_cb=self.execute_scan,
            auto_start=False,
        )
        self.server.start()

        rospy.loginfo("scan_environment_server: 就绪，等待扫描请求...")

    # ------------------------------------------------------------------
    #  相机内参回调
    # ------------------------------------------------------------------
    def camera_info_callback(self, msg):
        if self.camera_info_received:
            return
        K = msg.K
        self.fx = K[0]
        self.fy = K[4]
        self.cx = K[2]
        self.cy = K[5]
        self.img_width = msg.width
        self.img_height = msg.height
        self.camera_info_received = True
        rospy.loginfo(
            "scan_environment_server: 内参 K=[%.1f, %.1f, %.1f, %.1f] %dx%d",
            self.fx, self.fy, self.cx, self.cy, self.img_width, self.img_height
        )
        self.camera_info_sub.unregister()

    # ------------------------------------------------------------------
    #  深度图回调
    # ------------------------------------------------------------------
    def depth_callback(self, msg):
        if not self.camera_info_received:
            return
        try:
            img_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            depth = img_mm.astype(np.float32) * 0.001
            depth[(depth <= 0.0) | (depth > self.max_depth)] = 0.0
            with self.depth_lock:
                self.depth_img = depth
                self.depth_timestamp = msg.header.stamp
        except Exception as e:
            rospy.logerr_throttle(5.0, "scan_environment_server: cv_bridge 错误: %s", str(e))

    # ------------------------------------------------------------------
    #  采集一帧深度图（阻塞等待最新帧）
    # ------------------------------------------------------------------
    def _acquire_depth_frame(self, timeout=1.0):
        """等待并返回一帧有效的深度图。"""
        rate = rospy.Rate(100)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            with self.depth_lock:
                if self.depth_img is not None:
                    return self.depth_img.copy()
            if (rospy.Time.now() - start).to_sec() > timeout:
                return None
            rate.sleep()
        return None

    # ------------------------------------------------------------------
    #  平均 N 帧深度图以进一步降噪
    # ------------------------------------------------------------------
    def _average_depth_frames(self, N):
        """采集并逐像素平均 N 帧深度图。"""
        accum = None
        count = np.zeros((self.img_height, self.img_width), dtype=np.int32)
        for i in range(N):
            frame = self._acquire_depth_frame()
            if frame is None:
                rospy.logwarn("scan_environment_server: 采集第 %d/%d 帧失败", i + 1, N)
                continue
            if accum is None:
                accum = frame.copy().astype(np.float64)
            else:
                accum += frame.astype(np.float64)
            count += (frame > 0).astype(np.int32)

            if self.server.is_preempt_requested():
                return None

        if accum is None:
            return None

        depth = np.zeros_like(accum, dtype=np.float32)
        mask = count > 0
        depth[mask] = accum[mask].astype(np.float32) / count[mask].astype(np.float32)
        return depth

    # ------------------------------------------------------------------
    #  查询 TF 并返回 (平移, 旋转矩阵) 从 camera_frame 到 target_frame
    # ------------------------------------------------------------------
    def _lookup_camera_transform(self):
        """返回 camera_frame → target_frame 的 (translation, rotation_matrix)。"""
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                rospy.Time(0),
                rospy.Duration(2.0),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr("scan_environment_server: TF 查询失败: %s", str(e))
            return None, None

        trans = np.array([t.transform.translation.x,
                          t.transform.translation.y,
                          t.transform.translation.z])
        q = t.transform.rotation
        R = tr.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        return trans, R

    # ------------------------------------------------------------------
    #  将单个点和法向量从 camera_frame 变换到 target_frame
    # ------------------------------------------------------------------
    def _transform_point_and_normal(self, point_cam, normal_cam, trans, R):
        """返回 (point_base, normal_base) 均在 target_frame 下。"""
        point_base = trans + R @ point_cam
        normal_base = R @ normal_cam
        n = np.linalg.norm(normal_base)
        if n > 1e-9:
            normal_base = normal_base / n
        return point_base, normal_base

    # ------------------------------------------------------------------
    #  执行扫描：核心算法
    # ------------------------------------------------------------------
    def execute_scan(self, goal):
        """
        动作目标回调。
        1. 采集/平均深度帧
        2. 两遍网格扫描：PCA 法向量 + 空间平滑
        3. TF 变换到 target_frame
        4. 构造姿态，发布 MarkerArray
        5. 返回结果
        """
        # ---- 解析参数：goal 中的非零值覆盖 ROS param ----
        grid_r = goal.grid_rows if goal.grid_rows > 0 else self.grid_rows
        grid_c = goal.grid_cols if goal.grid_cols > 0 else self.grid_cols
        win_sz = goal.window_size if goal.window_size > 0 else self.window_size
        max_d = goal.max_depth if goal.max_depth > 0.0 else self.max_depth

        rospy.loginfo(
            "scan_environment_server: 开始扫描 grid=%dx%d win=%d max_d=%.1f avg=%d...",
            grid_r, grid_c, win_sz, max_d, self.scan_frame_average
        )

        # ---- 采集深度帧 ----
        feedback = ScanEnvironmentFeedback()
        feedback.progress = 0.05
        self.server.publish_feedback(feedback)

        if self.scan_frame_average > 1:
            depth = self._average_depth_frames(self.scan_frame_average)
        else:
            depth = self._acquire_depth_frame()

        if depth is None or self.server.is_preempt_requested():
            self.server.set_preempted()
            rospy.loginfo("scan_environment_server: 扫描被抢占或深度采集失败")
            return

        H, W = depth.shape
        if H == 0 or W == 0:
            self.server.set_aborted(ScanEnvironmentResult(message="depth image empty"))
            return

        # ---- TF 查询 ----
        feedback.progress = 0.1
        self.server.publish_feedback(feedback)

        trans, R_cam2base = self._lookup_camera_transform()
        if trans is None or self.server.is_preempt_requested():
            self.server.set_preempted()
            return

        # ================================================================
        #  第 1 遍：PCA 计算原始法向量和 3D 起点（camera 坐标系）
        # ================================================================
        half_win = win_sz // 2
        cell_h = H / float(grid_r)
        cell_w = W / float(grid_c)

        normals_2d = [[None] * grid_c for _ in range(grid_r)]
        origins_2d = [[None] * grid_c for _ in range(grid_r)]

        total_cells = float(grid_r * grid_c)
        processed = 0

        for r in range(grid_r):
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                return

            for c in range(grid_c):
                processed += 1
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

                # ---- 反投影 ----
                cols, rows = np.meshgrid(np.arange(c0, c1), np.arange(r0, r1))
                u = cols[valid_mask].astype(np.float64)
                v = rows[valid_mask].astype(np.float64)
                d = patch[valid_mask].astype(np.float64)

                x = (u - self.cx) * d / self.fx
                y = (v - self.cy) * d / self.fy
                z = d
                pts = np.stack([x, y, z], axis=1)

                # ---- SOR 去噪 ----
                pts, _ = sor_filter(pts, k=self.sor_k, stddev_mult=self.sor_stddev)
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
                # 方向一致性：指离相机（camera 光心帧 Z 向前，应指 +Z）
                if normal[2] < 0.0:
                    normal = -normal

                # ---- 3D 起点 ----
                ox = (uc - self.cx) * center_depth / self.fx
                oy = (vc - self.cy) * center_depth / self.fy
                oz = center_depth

                normals_2d[r][c] = normal
                origins_2d[r][c] = np.array([ox, oy, oz])

            # 每行反馈一次进度
            feedback.progress = 0.1 + 0.6 * (float(processed) / total_cells)
            self.server.publish_feedback(feedback)

        # ================================================================
        #  第 2 遍：空间邻域加权平滑 + TF 变换 + 构造姿态
        # ================================================================
        sr = self.smooth_radius
        sharp = self.smooth_sharpness

        samples = []
        markers = MarkerArray()

        # DELETEALL 清除上一轮可视化
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        delete_all.ns = "scan_normals"
        markers.markers.append(delete_all)

        marker_id = 0
        processed = 0

        for r in range(grid_r):
            for c in range(grid_c):
                processed += 1
                normal_cam = normals_2d[r][c]
                origin_cam = origins_2d[r][c]
                if normal_cam is None or origin_cam is None:
                    continue

                # ---- 空间加权平滑（camera 坐标系下） ----
                n_smooth = np.zeros(3)
                total_w = 0.0

                for dr in range(-sr, sr + 1):
                    for dc in range(-sr, sr + 1):
                        nr, nc = r + dr, c + dc
                        if nr < 0 or nr >= grid_r or nc < 0 or nc >= grid_c:
                            continue
                        nb = normals_2d[nr][nc]
                        if nb is None:
                            continue
                        if sharp > 0:
                            w = max(0.0, float(np.dot(normal_cam, nb))) ** sharp
                        else:
                            w = 1.0
                        n_smooth += w * nb
                        total_w += w

                if total_w > 0.0:
                    n_smooth /= total_w
                    n_smooth /= np.linalg.norm(n_smooth)
                else:
                    n_smooth = normal_cam

                # ---- TF 变换到 target_frame ----
                pos_base, norm_base = self._transform_point_and_normal(
                    origin_cam, n_smooth, trans, R_cam2base
                )

                # ---- 构造 6D 姿态 ----
                quat = normal_to_quaternion(norm_base)

                pose = Pose()
                pose.position.x = pos_base[0]
                pose.position.y = pos_base[1]
                pose.position.z = pos_base[2]
                pose.orientation = quat
                samples.append(pose)

                # ---- 构造 ARROW Marker（base_link 下可视化） ----
                tip_base = pos_base + norm_base * self.arrow_length

                marker = Marker()
                marker.header.frame_id = self.target_frame
                marker.header.stamp = self.depth_timestamp
                marker.ns = "scan_normals"
                marker.id = marker_id
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.points = [
                    Point(x=pos_base[0], y=pos_base[1], z=pos_base[2]),
                    Point(x=tip_base[0], y=tip_base[1], z=tip_base[2]),
                ]
                marker.scale.x = self.shaft_diam
                marker.scale.y = self.head_diam
                marker.scale.z = self.head_len
                marker.color.r = max(0.0, min(1.0, abs(norm_base[0])))
                marker.color.g = max(0.0, min(1.0, abs(norm_base[1])))
                marker.color.b = max(0.0, min(1.0, abs(norm_base[2])))
                marker.color.a = 0.85
                marker.lifetime = rospy.Duration(0)
                markers.markers.append(marker)
                marker_id += 1

            feedback.progress = 0.7 + 0.25 * (float(processed) / total_cells)
            self.server.publish_feedback(feedback)

        # ================================================================
        #  发布可视化并返回结果
        # ================================================================
        self.marker_pub.publish(markers)

        result = ScanEnvironmentResult()
        result.samples = samples
        result.message = "scan complete: {} sample points".format(len(samples))
        self.server.set_succeeded(result)

        rospy.loginfo(
            "scan_environment_server: 扫描完成，共 %d 个采样点", len(samples)
        )


# ======================================================================
#  主函数
# ======================================================================
def main():
    rospy.init_node("scan_environment_server")
    ScanEnvironmentServer()
    rospy.spin()


if __name__ == "__main__":
    main()
