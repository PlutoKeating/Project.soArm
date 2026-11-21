"""跟随用到的两个小工具：自适应低通滤波，和加速度连续的目标跟踪。"""

from __future__ import annotations

import math

import numpy as np


class OneEuroFilter:
    """One Euro 滤波器（Casiez 等，2012）：信号变化慢时滤得重，变化快时滤得轻。

    固定系数的低通只能二选一：滤得重，手不动时稳但手一快就滞后；滤得轻，跟得上但
    检测噪声会直接传到手臂。这个滤波器让截止频率随信号自身的变化速度升高：
    cutoff = min_cutoff + beta * |速度|。同时给出平滑后的速度，用作前馈和预判。
    输入可以是向量，各分量共用一个截止频率（按速度的模长算）。
    """

    def __init__(self, min_cutoff: float, beta: float, d_cutoff: float):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.value: np.ndarray | None = None
        self.velocity = np.zeros(1)
        self._t = 0.0
        self._raw: np.ndarray | None = None

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def update(self, x: np.ndarray, t: float) -> np.ndarray:
        x = np.asarray(x, float)
        if self.value is None:
            self.value, self.velocity, self._t, self._raw = x.copy(), np.zeros_like(x), t, x.copy()
            return self.value
        dt = max(t - self._t, 1e-3)
        self._t = t
        # 速度用相邻两次原始观测之差来估。原论文用“新观测减上一次滤波后的值”，那样只为调截止
        # 频率够用；但滤波后的值是滞后的，差值会被系统性放大（匀速 60°/秒 会估成 90 多），
        # 不能直接拿去做前馈
        raw_velocity = (x - self._raw) / dt
        self._raw = x.copy()
        a_d = self._alpha(self.d_cutoff, dt)
        self.velocity = self.velocity + a_d * (raw_velocity - self.velocity)
        cutoff = self.min_cutoff + self.beta * float(np.linalg.norm(self.velocity))
        a = self._alpha(cutoff, dt)
        self.value = self.value + a * (x - self.value)
        return self.value


def track_second_order(
    pos: np.ndarray,
    vel: np.ndarray,
    target: np.ndarray,
    target_vel: np.ndarray,
    dt: float,
    smooth_time: float,
    max_speed: float,
    max_accel: float,
    substeps: int = 4,
) -> tuple[np.ndarray, np.ndarray]:
    """临界阻尼的二阶跟踪：让 (pos, vel) 平滑地追上以 target_vel 运动的 target。

    与“比例控制 + 限速”相比：速度是连续的，不会每拍给舵机一个速度阶跃；临界阻尼所以
    不超调；带目标速度前馈，所以目标匀速运动时没有固定的滞后。smooth_time 越小追得越紧。
    """
    omega = 2.0 / max(smooth_time, 1e-3)
    h = dt / substeps
    pos, vel = np.asarray(pos, float).copy(), np.asarray(vel, float).copy()
    tgt = np.asarray(target, float).copy()
    for _ in range(substeps):
        accel = -2.0 * omega * (vel - target_vel) - omega * omega * (pos - tgt)
        accel = np.clip(accel, -max_accel, max_accel)
        vel = np.clip(vel + accel * h, -max_speed, max_speed)
        pos = pos + vel * h
        tgt = tgt + target_vel * h
    return pos, vel
