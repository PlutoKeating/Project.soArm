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
