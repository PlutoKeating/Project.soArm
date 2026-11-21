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
