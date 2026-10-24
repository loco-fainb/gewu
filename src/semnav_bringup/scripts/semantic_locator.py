#!/usr/bin/env python3
"""语义目标定位节点 v3：几何法 + 尺寸先验 + 多重几何门槛。

v2 -> v3 的改动（每条都对应自检发现的问题）：

P1 去掉硬编码相机位姿。v2 里 `CAM_X = 0.349/2`、`cam_z = 0.0125+0.50`
   是写死的，params.xacro 一改支架高度节点会静默出错。
   v3 改为把点云建在 depth_camera_optical_frame 下，用 TF 变换到目标坐标系，
   顺便把机器人 yaw 旋转也交给 TF，少一处手算。

P2 启用 planarity 门槛。实测冰箱平面性 0.003~0.017，墙面 0.16~0.28，
   相差一个数量级，是最强的真/伪信号，v2 却只打印不用。

P3 启用 z_min 落地门槛。这是修主卧衣柜误检的关键：那次的候选 z_min=+0.62，
   根本不是落地柜体（下半被遮挡只看到上段），不可信。v2 缺这道门槛时，
   衣柜误差分 0.102 与冰箱类 0.275 的差距不足以保证稳定拒绝。

P4 声明 use_sim_time。v2 未声明，TF 时间戳与节点时钟不同源；
   在 AMCL 下靠 Time() 兜底只是碰巧能用。

P7 所有门槛参数化，launch 全部暴露。

定位口径（重要，勿改）：
  输出的是**物体朝向相机那一面的中心点**，不是空间质心。
  深度相机只能看到正面，看不到背面。冰箱半深 0.36 m，
  因此正面点与质心差约 0.36 m 属正常，不是误差。
  评测必须用「到真值 AABB 的表面距离」，不能用质心距离。
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

CAMERA_FRAME = 'depth_camera_optical_frame'


@dataclass
class Candidate:
    """一个竖直面候选。坐标全部在目标坐标系（map 或 odom）下。"""
    n_px: int
    dist: float
    width: float
    height: float
    z_min: float
    z_max: float
    center: tuple[float, float, float]
    planarity: float
    normal: tuple[float, float, float]


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """四元数 -> 3x3 旋转矩阵。"""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


class GeomDetector:
    """几何检测器：纯计算，不依赖 ROS。

    输入深度图 + 相机内参 + 相机坐标系到目标坐标系的变换，
    输出目标坐标系下的竖直面候选。
    """

    def __init__(self, catalog: dict, cfg: dict):
        self.cat = catalog
        self.min_d = cfg['min_depth']
        self.max_d = cfg['max_depth']
        self.min_area = cfg['min_area_px']
        self.normal_z_tol = cfg['normal_z_tol']
        self.depth_jump = cfg['depth_jump']
        self.plane_tol = cfg['max_planarity']

    def _to_target(self, depth, fx, fy, cx, cy, R, t) -> np.ndarray:
        """深度图 -> 目标坐标系点云 (H,W,3)。

        光学系约定（REP-103）：x 右、y 下、z 前。
        """
        H, W = depth.shape
        v, u = np.mgrid[0:H, 0:W].astype(np.float32)
        d = depth.astype(np.float32)
        p_opt = np.stack([(u - cx) / fx * d,
                          (v - cy) / fy * d,
                          d], axis=-1)
        return p_opt.astype(np.float64) @ R.T + t

    def _normals(self, pts: np.ndarray) -> np.ndarray:
        du = np.zeros_like(pts)
        du[:, 1:-1] = pts[:, 2:] - pts[:, :-2]
        dv = np.zeros_like(pts)
        dv[1:-1, :] = pts[2:, :] - pts[:-2, :]
        nn = np.cross(du, dv)
        norm = np.linalg.norm(nn, axis=-1, keepdims=True)
        with np.errstate(invalid='ignore', divide='ignore'):
            return np.where(norm > 1e-9, nn / norm, 0.0)

    def find(self, depth, fx, fy, cx, cy, R, t) -> list[Candidate]:
        import cv2

        valid = np.isfinite(depth) & (depth > self.min_d) & (depth < self.max_d)
        pts = self._to_target(np.where(valid, depth, 0.0), fx, fy, cx, cy, R, t)
        nrm = self._normals(pts)

        # 竖直面：法向量水平（目标系 z 朝上，故 |n_z| 小）
        vertical = valid & (np.abs(nrm[..., 2]) < self.normal_z_tol)
        # 只保留合理高度范围内的点（排除天花板）
        vertical &= (pts[..., 2] > -0.6) & (pts[..., 2] < 3.2)

        d = depth.astype(np.float32)
        dx = np.abs(np.diff(d, axis=1, prepend=d[:, :1]))
        dy = np.abs(np.diff(d, axis=0, prepend=d[:1, :]))
        vertical &= ~((dx > self.depth_jump) | (dy > self.depth_jump) | ~valid)

        n_lab, lab = cv2.connectedComponents(vertical.astype(np.uint8), connectivity=8)

        out: list[Candidate] = []
        for i in range(1, n_lab):
            m = (lab == i)
            cnt = int(m.sum())
            if cnt < self.min_area:
                continue
            p = pts[m]
            dd = depth[m]
            c = p.mean(axis=0)
            _, s, vt = np.linalg.svd(p - c, full_matrices=False)
            n = vt[-1]
            if n[2] < 0:
                n = -n
            if abs(n[2]) > self.normal_z_tol:
                continue

            e_v = np.array([0.0, 0.0, 1.0])
            e_u = np.cross(e_v, n)
            nu = np.linalg.norm(e_u)
            if nu < 1e-6:
                continue
            e_u /= nu

            pu = (p - c) @ e_u
            pv = (p - c) @ e_v
            cu = float((pu.max() + pu.min()) / 2)
            # 面中心在地面上的落点（目标系，z 压到 0）
            ground = c + e_u * cu
            ground[2] = 0.0

            out.append(Candidate(
                n_px=cnt,
                dist=float(np.median(dd)),
                width=float(pu.max() - pu.min()),
                height=float(pv.max() - pv.min()),
                z_min=float(p[:, 2].min()),
                z_max=float(p[:, 2].max()),
                center=(float(ground[0]), float(ground[1]), 0.0),
                planarity=float(s[-1] / max(s[0], 1e-9)),
                normal=tuple(float(v) for v in n),
            ))
        out.sort(key=lambda r: -r.n_px)
        return out

    def reject_reason(self, c: Candidate) -> str | None:
        """几何可用性门槛（P2/P3）。返回 None 表示通过。"""
        if c.planarity > self.plane_tol:
            return f'非平面 {c.planarity:.4f}>{self.plane_tol}'
        if c.z_min > self.ground_z_max:
            return f'未落地 z_min={c.z_min:+.3f}>{self.ground_z_max}'
        if c.z_min < self.ground_z_min:
            return f'高度异常 z_min={c.z_min:+.3f}<{self.ground_z_min}'
        return None

    def classify(self, c: Candidate) -> list[tuple[float, str, dict]]:
        """按 (宽, 高) 匹配类别。返回 [(误差分, class, entry)] 升序。"""
        out = []
        for cls, entries in self.cat.items():
            best = None
            for e in entries:
                for hw in e['dims']:
                    ew = abs(c.width - hw) / max(hw, 1e-6)
                    eh = abs(c.height - e['height']) / max(e['height'], 1e-6)
                    sc = (ew + eh) / 2
                    if best is None or sc < best[0]:
                        best = (sc, cls, {**e, 'matched_w': hw})
            if best:
                out.append(best)
        out.sort(key=lambda x: x[0])
        return out


def load_catalog(path: str) -> dict:
    a = yaml.safe_load(open(path, encoding='utf-8'))
    cat: dict[str, list[dict]] = {}
    for o in a.get('objects') or []:
        sz = o['size']
        cat.setdefault(o['class'], []).append({
            'model': o['model'],
            'dims': sorted([sz[0], sz[1]]),
            'height': sz[2],
            'instance': o.get('label', ''),
        })
    return cat


class SemanticLocatorNode(Node):
    def __init__(self):
        super().__init__('semantic_locator')

        # P4: use_sim_time 由 rclpy 内置声明，此处不可重复声明，
        # 否则抛 ParameterAlreadyDeclaredException（实测踩过）。
        # 通过 launch 的 parameters=[{'use_sim_time': True}] 传入即可。
        self.declare_parameter('layout_yaml', '')
        self.declare_parameter('target_class', 'refrigerator')
        # 判定门槛（P7: 全部可调）
        self.declare_parameter('min_score', 0.15)
        self.declare_parameter('min_margin', 0.02)
        self.declare_parameter('max_planarity', 0.05)
        self.declare_parameter('ground_z_min', -0.15)
        self.declare_parameter('ground_z_max', 0.35)
        self.declare_parameter('min_area_px', 2500)
        self.declare_parameter('min_depth', 0.15)
        self.declare_parameter('max_depth', 4.5)
        self.declare_parameter('normal_z_tol', 0.45)
        self.declare_parameter('depth_jump', 0.06)
        # 坐标系
        self.declare_parameter('target_frame', 'auto')
        self.declare_parameter('max_hz', 2.0)
        self.declare_parameter('backend', 'auto')

        gp = self.get_parameter
        self.target = gp('target_class').value
        self.min_score = float(gp('min_score').value)
        self.min_margin = float(gp('min_margin').value)
        self.frame_pref = gp('target_frame').value
        self.backend = gp('backend').value
        self.min_period = 1.0 / max(0.1, float(gp('max_hz').value))

        cfg = {
            'min_depth': float(gp('min_depth').value),
            'max_depth': float(gp('max_depth').value),
            'min_area_px': int(gp('min_area_px').value),
            'normal_z_tol': float(gp('normal_z_tol').value),
            'depth_jump': float(gp('depth_jump').value),
            'max_planarity': float(gp('max_planarity').value),
        }
        cat = load_catalog(gp('layout_yaml').value)
        self.det = GeomDetector(cat, cfg)
        # P2/P3 门槛值挂到实例上
        self.det.ground_z_min = float(gp('ground_z_min').value)
        self.det.ground_z_max = float(gp('ground_z_max').value)

        self.get_logger().info(
            f'清单 {len(cat)} 类 / {sum(len(v) for v in cat.values())} 实例，'
            f'目标={self.target}，frame={self.frame_pref}')
        self.get_logger().info(
            f'门槛: score<={self.min_score} margin>={self.min_margin} '
            f'planarity<={self.det.plane_tol} '
            f'z_min∈[{self.det.ground_z_min},{self.det.ground_z_max}] '
            f'area>={self.det.min_area}')

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, '/depth_camera/depth', self.on_depth, qos)
        self.create_subscription(CameraInfo, '/depth_camera/camera_info', self.on_info, qos)

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.pub = self.create_publisher(String, '/semnav/detections', 10)

        self.info = None
        self.resolved_frame = None
        self.last_proc = 0.0
        self.last_warn = 0.0
        self.best_score = float('inf')
        self.best_pose = None
        self.frames = 0
        self.hits = 0
        self.rejected: dict[str, int] = {}
        self.get_logger().info('节点就绪（v3，TF 变换 + 几何门槛）')

    def on_info(self, msg):
        self.info = msg

    def _resolve_frame(self):
        if self.resolved_frame:
            return self.resolved_frame
        cands = ([self.frame_pref] if self.frame_pref != 'auto' else ['map', 'odom'])
        for f in cands:
            try:
                self.tf_buf.lookup_transform(f, CAMERA_FRAME, Time())
                self.resolved_frame = f
                self.get_logger().info(f'坐标系: {f} <- {CAMERA_FRAME}')
                return f
            except Exception:
                continue
        return None

    def _bump(self, key: str):
        self.rejected[key] = self.rejected.get(key, 0) + 1

    def on_depth(self, msg):
        now = time.monotonic()
        if now - self.last_proc < self.min_period:
            return
        self.last_proc = now
        if self.info is None:
            return

        frame = self._resolve_frame()
        if frame is None:
            if now - self.last_warn > 5.0:
                self.get_logger().warn(f'TF 不可用（等待 {CAMERA_FRAME} -> map/odom）')
                self.last_warn = now
            return

        # P1: 相机位姿完全来自 TF，不再硬编码
        try:
            tf = self.tf_buf.lookup_transform(frame, CAMERA_FRAME, Time())
        except Exception as e:
            if now - self.last_warn > 5.0:
                self.get_logger().warn(f'TF 查询失败: {e}')
                self.last_warn = now
            return

        tr = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_matrix(q.x, q.y, q.z, q.w)
        t = np.array([tr.x, tr.y, tr.z], dtype=np.float64)

        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width)
        k = self.info.k

        cands = self.det.find(depth, k[0], k[4], k[2], k[5], R, t)
        self.frames += 1

        best = None
        for c in cands:
            why = self.det.reject_reason(c)
            if why:
                self._bump(why.split()[0])
                continue
            ranked = self.det.classify(c)
            if not ranked:
                continue
            score, cls, entry = ranked[0]
            if cls != self.target:
                self._bump(f'类别={cls}')
                continue
            if score > self.min_score:
                self._bump(f'尺寸误差>{self.min_score}')
                continue
            margin = (ranked[1][0] - score) if len(ranked) > 1 else 1.0
            if margin < self.min_margin:
                self._bump('区分度不足')
                continue
            if best is None or score < best[0]:
                best = (score, cls, entry, c, margin)

        if best is None:
            return

        score, cls, entry, c, margin = best
        mx, my = c.center[0], c.center[1]

        self.hits += 1
        if score < self.best_score:
            self.best_score = score
            self.best_pose = (mx, my)

        self.pub.publish(String(data=json.dumps({
            'class': cls, 'target': self.target,
            'score': round(score, 4), 'margin': round(margin, 4),
            'distance': round(c.dist, 4),
            'width': round(c.width, 4), 'height': round(c.height, 4),
            'z_min': round(c.z_min, 4), 'z_max': round(c.z_max, 4),
            'n_px': c.n_px, 'planarity': round(c.planarity, 5),
            'pose': [round(mx, 4), round(my, 4)],
            'frame': frame,
        }, ensure_ascii=False)))

        self.get_logger().info(
            f'[{self.target}] score={score:.4f} 差={margin:.3f} '
            f'距={c.dist:.3f} 宽={c.width:.3f} 高={c.height:.3f} '
            f'z_min={c.z_min:+.3f} 平面性={c.planarity:.4f} '
            f'→ {frame}({mx:.3f},{my:.3f})')


def main(argv=None):
    rclpy.init(args=argv)
    node = SemanticLocatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        bs = node.best_score if node.best_score < float('inf') else 'n/a'
        node.get_logger().info(
            f'共 {node.frames} 帧, {node.hits} 命中, frame={node.resolved_frame}, '
            f'最优={bs}, 拒绝统计={node.rejected}')
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

