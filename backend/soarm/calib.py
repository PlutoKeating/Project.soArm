"""深度相机到机械臂基座的外参标定（触点法）。

做法：把夹爪尖移动到深度相机能看见的若干位置，每个位置在深度画面上点一下夹爪尖。
这样同一个物理点就有两组坐标：相机系下的（来自深度反投影）和基座系下的（来自正运动学）。
凑够不共面的 4 组以上，用 Kabsch 算法解出刚体变换 T_base_cam。

外参只在云台处于标定时的姿态下有效，所以结果里同时记录了云台预设角。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

MIN_SAMPLES = 4


def solve_rigid(cam_pts: np.ndarray, base_pts: np.ndarray) -> tuple[np.ndarray, float]:
    """求 T 使 base ≈ T @ cam。返回 (4x4 变换, RMS 残差/米)。"""
    cam_pts, base_pts = np.asarray(cam_pts, float), np.asarray(base_pts, float)
    if len(cam_pts) < MIN_SAMPLES:
        raise ValueError(f"至少需要 {MIN_SAMPLES} 个采样点")
    cc, bc = cam_pts.mean(axis=0), base_pts.mean(axis=0)
    H = (cam_pts - cc).T @ (base_pts - bc)
    if np.linalg.matrix_rank(H) < 3:
        raise ValueError("采样点共面或共线，请在不同高度和远近多采几个点")
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = bc - R @ cc
    residual = base_pts - (cam_pts @ R.T + T[:3, 3])
    return T, float(np.sqrt((residual**2).sum(axis=1).mean()))


@dataclass
class BearingSample:
    """同一只手的一次配对观测：臂上腕部相机看到它的视线 + 深度相机测到它的 3D 位置。"""

    origin: np.ndarray  # 视线起点（腕部相机位置），臂基座系，米
    direction: np.ndarray  # 视线单位方向，臂基座系
    cam_point: np.ndarray  # 手在深度相机光学系下的位置，米


# 深度相机光学系 (x 右, y 下, z 前) 到臂基座系 (x 前, y 左, z 上) 的名义朝向
_NOMINAL_R = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
MIN_BEARING_SAMPLES = 10


def _rotvec_to_matrix(w: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(w))
    if angle < 1e-12:
        return np.eye(3)
    k = w / angle
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def _bearing_residuals(params, R0, origins, dirs, pts):
    """每个样本两到三个分量：手的位置偏离视线的角度（弧度）。"""
    R = R0 @ _rotvec_to_matrix(params[:3])
    v = pts @ R.T + params[3:] - origins
    along = np.einsum("ij,ij->i", v, dirs)
    perp = v - along[:, None] * dirs
    return (perp / np.maximum(np.linalg.norm(v, axis=1), 0.05)[:, None]).ravel()


def _gauss_newton(R0, t0, origins, dirs, pts, iters: int = 40):
    params = np.concatenate([np.zeros(3), t0])
    lam = 1e-3
    res = _bearing_residuals(params, R0, origins, dirs, pts)
    cost = float(res @ res)
    for _ in range(iters):
        J = np.empty((res.size, 6))
        for i in range(6):
            step = np.zeros(6)
            step[i] = 1e-5
            J[:, i] = (_bearing_residuals(params + step, R0, origins, dirs, pts) - res) / 1e-5
        delta = np.linalg.solve(J.T @ J + lam * np.eye(6), -J.T @ res)
        trial = params + delta
        trial_res = _bearing_residuals(trial, R0, origins, dirs, pts)
        trial_cost = float(trial_res @ trial_res)
        if trial_cost < cost:
            params, res, cost, lam = trial, trial_res, trial_cost, max(lam / 3, 1e-9)
            if float(np.linalg.norm(delta)) < 1e-9:
                break
        else:
            lam *= 5
    return R0 @ _rotvec_to_matrix(params[:3]), params[3:], cost


def solve_from_bearings(samples: list[BearingSample]) -> tuple[np.ndarray, float, int]:
    """由“视线 + 3D 点”配对求 T_base_cam。返回 (4x4 变换, 手偏离视线的 RMS 距离/米, 采用的样本数)。

    这是一个 PnP 问题：深度相机给出手的 3D 位置，臂上的腕部相机给出它的方向。
    从多个初始朝向出发做非线性最小二乘，取最好的；再剔除明显的离群样本重解一次
    （两台相机偶尔看到的不是同一只手）。
    """
    if len(samples) < MIN_BEARING_SAMPLES:
        raise ValueError(f"至少需要 {MIN_BEARING_SAMPLES} 个配对样本，当前 {len(samples)} 个")
    origins = np.array([s.origin for s in samples], float)
    dirs = np.array([s.direction for s in samples], float)
    pts = np.array([s.cam_point for s in samples], float)
    if np.linalg.matrix_rank(pts - pts.mean(axis=0), tol=0.02) < 3:
        raise ValueError("样本里手的位置几乎都在一个平面或一条线上，请在不同远近、高低、左右多移动一些")

    def best_fit(o, d, p):
        best = None
        for pitch in (0.0, 25.0, 50.0, 75.0):  # 相机低头的角度
            for yaw in (-40.0, 0.0, 40.0):
                cp, sp = np.cos(np.deg2rad(pitch)), np.sin(np.deg2rad(pitch))
                cy, sy = np.cos(np.deg2rad(yaw)), np.sin(np.deg2rad(yaw))
                Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
                Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
                R0 = Rz @ Ry @ _NOMINAL_R
                # 初始平移：让点云中心落在视线上离起点 0.35 米处
                t0 = (o + 0.35 * d).mean(axis=0) - (p @ R0.T).mean(axis=0)
                R, t, cost = _gauss_newton(R0, t0, o, d, p)
                if best is None or cost < best[2]:
                    best = (R, t, cost)
        return best

    def distances(R, t, o, d, p):
        v = p @ R.T + t - o
        return np.linalg.norm(v - np.einsum("ij,ij->i", v, d)[:, None] * d, axis=1)

    R, t, _ = best_fit(origins, dirs, pts)
    dist = distances(R, t, origins, dirs, pts)
    keep = dist <= max(3.0 * float(np.median(dist)), 0.02)
    if MIN_BEARING_SAMPLES <= int(keep.sum()) < len(samples):
        origins, dirs, pts = origins[keep], dirs[keep], pts[keep]
        R, t, _ = best_fit(origins, dirs, pts)
        dist = distances(R, t, origins, dirs, pts)
    # 手必须在腕部相机前方，否则是镜像解
    if float(np.mean(np.einsum("ij,ij->i", pts @ R.T + t - origins, dirs))) <= 0:
        raise ValueError("求解结果不合理（手落在相机后方），请重新采集")
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T, float(np.sqrt((dist**2).mean())), len(pts)


@dataclass
class BodyObservation:
    """机械臂摆在某个姿态时，深度相机看到的属于手臂的点。"""

    skeleton: np.ndarray  # 该姿态下的骨架点，臂基座系，(K, 3)
    cam_points: np.ndarray  # 手臂表面的点，相机光学系，(N, 3)


def _segment_distance(points: np.ndarray, skeleton: np.ndarray) -> np.ndarray:
    """每个点到骨架折线的最近距离。"""
    best = np.full(len(points), np.inf)
    for a, b in zip(skeleton[:-1], skeleton[1:]):
        ab = b - a
        length2 = float(ab @ ab)
        if length2 < 1e-10:
            continue
        t = np.clip((points - a) @ ab / length2, 0.0, 1.0)
        best = np.minimum(best, np.linalg.norm(points - (a + t[:, None] * ab), axis=1))
    return best


def solve_from_body(
    observations: list[BodyObservation],
    pitch_down_deg: float,
    link_radius: float = 0.022,
) -> tuple[np.ndarray, float, int]:
    """用机械臂自身当标定物求 T_base_cam。返回 (4x4 变换, 点到手臂表面的 RMS 距离/米, 采用的点数)。

    让机械臂摆若干姿态，深度相机每次看到的手臂点云，换到基座系后都应该贴在该姿态的
    骨架周围。对 6 个外参做多初值的非线性最小二乘。相机低头的角度可以从地面测出来，
    用它缩小初值范围；相机大致在臂的上方、朝前，这一点也用来排初值。
    """
    if len(observations) < 4:
        raise ValueError("至少需要 4 个姿态")
    skeletons = [o.skeleton for o in observations]
    clouds = [np.asarray(o.cam_points, float) for o in observations]
    if sum(len(c) for c in clouds) < 200:
        raise ValueError("手臂点云太少：相机可能看不到手臂")
    cap = 0.08  # 残差封顶：远处的点（路人、线缆）不让它们主导结果

    def residuals(params, R0):
        R = R0 @ _rotvec_to_matrix(params[:3])
        out = []
        for skel, cloud in zip(skeletons, clouds):
            d = _segment_distance(cloud @ R.T + params[3:], skel) - link_radius
            out.append(np.clip(d, -cap, cap))
        return np.concatenate(out)

    def refine(R0, t0, iters=30):
        params = np.concatenate([np.zeros(3), t0])
        lam = 1e-2
        res = residuals(params, R0)
        cost = float(res @ res)
        for _ in range(iters):
            J = np.empty((res.size, 6))
            for i in range(6):
                step = np.zeros(6)
                step[i] = 1e-4
                J[:, i] = (residuals(params + step, R0) - res) / 1e-4
            delta = np.linalg.solve(J.T @ J + lam * np.eye(6), -J.T @ res)
            trial = params + delta
            trial_res = residuals(trial, R0)
            trial_cost = float(trial_res @ trial_res)
            if trial_cost < cost:
                params, res, cost, lam = trial, trial_res, trial_cost, max(lam / 3, 1e-8)
                if float(np.linalg.norm(delta)) < 1e-7:
                    break
            else:
                lam *= 4
                if lam > 1e6:
                    break
        return R0 @ _rotvec_to_matrix(params[:3]), params[3:], cost

    pitch = np.deg2rad(pitch_down_deg)
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    best = None
    for yaw in (-30.0, -10.0, 10.0, 30.0):
        cy, sy = np.cos(np.deg2rad(yaw)), np.sin(np.deg2rad(yaw))
        R0 = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ Ry @ _NOMINAL_R
        for x in (-0.15, 0.0, 0.15):
            for y in (-0.35, -0.15, 0.15, 0.35):
                for z in (0.3, 0.5):
                    R, t, cost = refine(R0, np.array([x, y, z]), iters=12)
                    if best is None or cost < best[2]:
                        best = (R, t, cost)
    R, t, _ = refine(best[0], best[1], iters=60)
    dist = np.concatenate([
        _segment_distance(cloud @ R.T + t, skel) - link_radius for skel, cloud in zip(skeletons, clouds)
    ])
    inliers = np.abs(dist) < cap
    if inliers.mean() < 0.5:
        raise ValueError(f"只有 {inliers.mean():.0%} 的点能贴到手臂模型上，标定不可信")
    # 合理性：相机装在手臂上方、朝前下方看。对着一堆无关的点（比如路过的人）也可能凑出
    # 一个残差不大的解，但位置朝向会很离谱，在这里拦下
    if not 0.15 < t[2] < 1.0 or R[0, 2] < 0.2 or R[2, 2] > -0.1:
        raise ValueError(
            f"求出的相机位置 ({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}) 或朝向 {np.round(R[:, 2], 2).tolist()} 不合理："
            "相机应在手臂上方、朝前下方。多半是手臂没进入相机视野，点云里是别的东西"
        )
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T, float(np.sqrt((dist[inliers] ** 2).mean())), int(inliers.sum())


def verify_body(
    T: np.ndarray,
    skeletons: list[np.ndarray],
    depths: list[np.ndarray],
    intrinsics: tuple[float, float, float, float],
    link_radius: float = 0.022,
) -> tuple[float, float]:
    """反向验证外参：把每个姿态的手臂模型投影到深度图上，看深度图是否真的在那里看到了东西。

    返回 (模型点落在画面内的比例, 其中被深度证实的比例)。求解只保证“看到的点贴着模型”，
    这里检查“模型在的地方确实有点”，两头都成立才可信。
    """
    fx, fy, cx, cy = intrinsics
    T_inv = np.linalg.inv(T)
    total = inside = confirmed = 0
    for skel, depth in zip(skeletons, depths):
        h, w = depth.shape
        for a, b in zip(skel[2:-1], skel[3:]):  # 前臂与夹爪：会进入视野的部分
            for f in np.linspace(0.1, 0.9, 12):
                p = T_inv[:3, :3] @ (a + f * (b - a)) + T_inv[:3, 3]
                total += 1
                if p[2] <= 0.05:
                    continue
                u, v = int(round(fx * p[0] / p[2] + cx)), int(round(fy * p[1] / p[2] + cy))
                if not (2 <= u < w - 2 and 2 <= v < h - 2):
                    continue
                inside += 1
                patch = depth[v - 2 : v + 3, u - 2 : u + 3]
                valid = patch[patch > 0.05]
                if valid.size and abs(float(np.median(valid)) - (p[2] - link_radius)) < 0.05:
                    confirmed += 1
    return inside / max(total, 1), confirmed / max(inside, 1)


@dataclass
class ArmExtrinsic:
    T_base_cam: np.ndarray
    rms: float
    samples: int


@dataclass
class Calibration:
    head_preset: dict[str, float] | None = None  # 标定时的云台角 {"pan", "tilt"}
    arms: dict[str, ArmExtrinsic] = field(default_factory=dict)

    @classmethod
    def load(cls, path: Path) -> "Calibration":
        if not path.exists():
            return cls()
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        arms = {
            name: ArmExtrinsic(np.array(a["T_base_cam"], float), float(a["rms"]), int(a["samples"]))
            for name, a in (data.get("arms") or {}).items()
        }
        return cls(head_preset=data.get("head_preset"), arms=arms)

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "head_preset": self.head_preset,
            "arms": {
                name: {
                    "T_base_cam": [[round(float(v), 6) for v in row] for row in a.T_base_cam],
                    "rms": round(a.rms, 5),
                    "samples": a.samples,
                }
                for name, a in self.arms.items()
            },
        }
        path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")


class CalibSession:
    """采样暂存。每只臂各自一组点对。"""

    def __init__(self):
        self.samples: dict[str, list[tuple[np.ndarray, np.ndarray]]] = {}

    def add(self, arm: str, cam_pt: np.ndarray, base_pt: np.ndarray) -> int:
        self.samples.setdefault(arm, []).append((np.asarray(cam_pt, float), np.asarray(base_pt, float)))
        return len(self.samples[arm])

    def undo(self, arm: str) -> int:
        pts = self.samples.get(arm, [])
        if pts:
            pts.pop()
        return len(pts)

    def clear(self, arm: str) -> None:
        self.samples.pop(arm, None)

    def count(self, arm: str) -> int:
        return len(self.samples.get(arm, []))

    def solve(self, arm: str) -> ArmExtrinsic:
        pts = self.samples.get(arm, [])
        cam = np.array([p[0] for p in pts]).reshape(-1, 3)
        base = np.array([p[1] for p in pts]).reshape(-1, 3)
        T, rms = solve_rigid(cam, base)
        return ArmExtrinsic(T, rms, len(pts))
