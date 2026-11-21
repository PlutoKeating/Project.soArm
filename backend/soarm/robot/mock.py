"""无硬件时的替身：关节以有限速度趋向目标。用于在没有机械臂/电源时跑通整套控制台。"""

from __future__ import annotations

import time

from .base import JOINTS, Arm, Head

_MOCK_SPEED_DEG_S = 120.0


class MockArm(Arm):
    def __init__(self, name: str):
        super().__init__(name)
        # 模拟上电时停在折叠休息位，而不是初始位
        self._pos = {j: 0.0 for j in JOINTS}
        self._pos.update({"shoulder_lift": -95.0, "elbow_flex": 90.0, "wrist_flex": 60.0})
        self._target = dict(self._pos)
        self._last = time.monotonic()

    def connect(self) -> None:
        self.connected = True
        self.torque_enabled = True

    def disconnect(self) -> None:
        self.connected = False
        self.torque_enabled = False

    def _step(self) -> None:
        now = time.monotonic()
        dt, self._last = now - self._last, now
        if not self.torque_enabled:
            return
        max_step = _MOCK_SPEED_DEG_S * dt
        for j in JOINTS:
            err = self._target[j] - self._pos[j]
            self._pos[j] += max(-max_step, min(max_step, err))

    def read(self) -> dict[str, float]:
        self._step()
        return dict(self._pos)

    def write(self, target: dict[str, float]) -> None:
        self._step()
        self._target.update(target)

    def set_torque(self, enabled: bool) -> None:
        self.torque_enabled = enabled


class MockHead(Head):
    def __init__(self):
        super().__init__()
        self._pos = {"pan": 0.0, "tilt": 0.0}

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def read(self) -> dict[str, float]:
        return dict(self._pos)

    def write(self, target: dict[str, float]) -> None:
        self._pos.update(target)

    def limits(self) -> dict[str, tuple[float, float]]:
        return {"pan": (-60.0, 60.0), "tilt": (-40.0, 40.0)}
