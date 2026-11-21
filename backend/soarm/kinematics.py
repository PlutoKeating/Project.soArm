"""SO-101 运动学：解析 URDF 做正解，用阻尼最小二乘做 4 自由度逆解。

SO-101 只有 5 个臂关节，无法实现任意 6 维位姿，因此这里不解完整位姿，而是解一个
与机构匹配的 4 维任务：夹爪参考点位置 (x, y, z) + 夹爪指向的俯仰角 pitch。
参与求解的关节是 shoulder_pan / shoulder_lift / elbow_flex / wrist_flex；
wrist_roll 不影响这 4 维任务，由上层直接给定。

关节角单位统一为“度”，并假定与 LeRobot 校准后的角度同号同零位
（LeRobot 自带的运动学管线对 so101_new_calib.urdf 也是这样假定的）。
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

URDF_PATH = Path(__file__).parent / "assets" / "so101_new_calib.urdf"

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
IK_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]
TIP_LINK = "gripper_frame_link"
BASE_LINK = "base_link"


def _rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis / np.linalg.norm(axis)
    c, s = np.cos(angle), np.sin(angle)
    k = 1 - c
    return np.array(
        [
            [c + x * x * k, x * y * k - z * s, x * z * k + y * s],
            [y * x * k + z * s, c + y * y * k, y * z * k - x * s],
            [z * x * k - y * s, z * y * k + x * s, c + z * z * k],
        ]
    )


@dataclass
class _Joint:
    name: str
    fixed: bool
    origin: np.ndarray  # 4x4，父 link 到关节坐标系
    axis: np.ndarray
    lower_deg: float
    upper_deg: float


@dataclass
class IKResult:
    joints: dict[str, float]  # IK_JOINTS 的解（度）
    pos_error: float  # 位置残差（米）
    pitch_error: float  # 俯仰残差（度）


class ArmKinematics:
    """从 base_link 到 gripper_frame_link 的运动链。"""

    def __init__(self, urdf_path: Path = URDF_PATH):
        root = ET.parse(urdf_path).getroot()
        by_child: dict[str, tuple[str, _Joint]] = {}
        for j in root.findall("joint"):
            origin_el = j.find("origin")
            xyz = np.fromstring(origin_el.get("xyz", "0 0 0"), sep=" ")
            rpy = np.fromstring(origin_el.get("rpy", "0 0 0"), sep=" ")
            T = np.eye(4)
            T[:3, :3] = _rpy_to_matrix(rpy)
            T[:3, 3] = xyz
            axis_el = j.find("axis")
            axis = np.fromstring(axis_el.get("xyz"), sep=" ") if axis_el is not None else np.zeros(3)
            lim = j.find("limit")
            fixed = j.get("type") == "fixed"
            lower = float(np.rad2deg(float(lim.get("lower")))) if lim is not None else 0.0
            upper = float(np.rad2deg(float(lim.get("upper")))) if lim is not None else 0.0
            by_child[j.find("child").get("link")] = (
                j.find("parent").get("link"),
                _Joint(j.get("name"), fixed, T, axis, lower, upper),
            )

        chain: list[_Joint] = []
        link = TIP_LINK
        while link != BASE_LINK:
            parent, joint = by_child[link]
            chain.append(joint)
            link = parent
        self.chain = list(reversed(chain))
        self.limits = {j.name: (j.lower_deg, j.upper_deg) for j in self.chain if not j.fixed}

    # ---------- 正解 ----------
    def _frames(self, joints_deg: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:
        """返回 (夹爪参考点位姿 4x4, wrist_roll 关节原点位置)。"""
        T = np.eye(4)
        roll_origin = np.zeros(3)
        for j in self.chain:
            T = T @ j.origin
            if j.name == "wrist_roll":
                roll_origin = T[:3, 3].copy()
            if not j.fixed:
                R = np.eye(4)
                R[:3, :3] = _axis_angle(j.axis, np.deg2rad(joints_deg.get(j.name, 0.0)))
                T = T @ R
        return T, roll_origin

    def fk(self, joints_deg: dict[str, float]) -> np.ndarray:
        """夹爪参考点在基座系下的 4x4 位姿。"""
        return self._frames(joints_deg)[0]

    def tip_and_pitch(self, joints_deg: dict[str, float]) -> tuple[np.ndarray, float]:
        """夹爪参考点位置（米）与夹爪指向的俯仰角（度，抬头为正）。"""
        T, roll_origin = self._frames(joints_deg)
        tip = T[:3, 3]
        a = tip - roll_origin
        a /= np.linalg.norm(a)
        pitch = np.rad2deg(np.arctan2(a[2], np.hypot(a[0], a[1])))
        return tip.copy(), float(pitch)

    def approach_dir(self, joints_deg: dict[str, float]) -> np.ndarray:
        """夹爪指向（单位向量，基座系）。"""
        T, roll_origin = self._frames(joints_deg)
        a = T[:3, 3] - roll_origin
        return a / np.linalg.norm(a)

    # ---------- 逆解 ----------
    def _task(self, q: np.ndarray, other: dict[str, float]) -> np.ndarray:
        joints = dict(other)
        joints.update(dict(zip(IK_JOINTS, q)))
        tip, pitch = self.tip_and_pitch(joints)
        return np.array([tip[0], tip[1], tip[2], np.deg2rad(pitch) * self.pitch_weight])

    pitch_weight = 0.08  # 1 rad 俯仰误差折算为 8 cm 位置误差

    def ik(
        self,
        target_pos: np.ndarray,
        target_pitch_deg: float,
        seed_deg: dict[str, float],
        max_iters: int = 30,
        damping: float = 3e-4,
        max_step_deg: float = 8.0,
        limits: dict[str, tuple[float, float]] | None = None,
    ) -> IKResult:
        """从 seed 出发迭代求解。上层应检查残差，残差大说明目标不可达。"""
        q = np.array([seed_deg.get(n, 0.0) for n in IK_JOINTS], dtype=float)
        limits = limits or self.limits  # 可传入某只臂自己的（更窄的）限位
        lo = np.array([limits[n][0] for n in IK_JOINTS])
        hi = np.array([limits[n][1] for n in IK_JOINTS])
        target = np.array([*target_pos, np.deg2rad(target_pitch_deg) * self.pitch_weight])
        eps = 0.05  # 数值雅可比的差分步长（度）
        for _ in range(max_iters):
            cur = self._task(q, seed_deg)
            err = target - cur
            if np.linalg.norm(err) < 2e-4:
                break
            J = np.zeros((4, 4))
            for i in range(4):
                dq = q.copy()
                dq[i] += eps
                J[:, i] = (self._task(dq, seed_deg) - cur) / eps
            step = J.T @ np.linalg.solve(J @ J.T + (damping**2) * np.eye(4), err)
            biggest = np.max(np.abs(step))
            if biggest > max_step_deg:
                step *= max_step_deg / biggest
            q = np.clip(q + step, lo, hi)
        joints = dict(seed_deg)
        joints.update(dict(zip(IK_JOINTS, q)))
        tip, pitch = self.tip_and_pitch(joints)
        return IKResult(
            joints={n: float(v) for n, v in zip(IK_JOINTS, q)},
            pos_error=float(np.linalg.norm(tip - np.asarray(target_pos))),
            pitch_error=float(abs(pitch - target_pitch_deg)),
        )

    @staticmethod
    def clamp(joints_deg: dict[str, float], limits: dict[str, tuple[float, float]]) -> dict[str, float]:
        return {
            n: float(np.clip(v, *limits[n])) if n in limits else v for n, v in joints_deg.items()
        }
