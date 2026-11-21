"""机械臂与云台的硬件抽象。控制层只依赖这里的接口，不直接碰 LeRobot。"""

from __future__ import annotations

from abc import ABC, abstractmethod

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


class Arm(ABC):
    """单只 SO-100/101 follower。5 个臂关节单位为度，gripper 为开合度 0-100。"""

    def __init__(self, name: str):
        self.name = name
        self.connected = False
        self.torque_enabled = False
        self.error: str | None = None
        # 连接时力矩是否本来就是关闭的（即手臂正无力停放着）。控制层据此决定
        # 能否把连接时的姿态当作休息位
        self.found_resting = True

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def read(self) -> dict[str, float]:
        """读取 6 个关节的当前位置。"""

    @abstractmethod
    def write(self, target: dict[str, float]) -> None:
        """下发 6 个关节的目标位置。"""

    @abstractmethod
    def set_torque(self, enabled: bool) -> None: ...

    def limits(self) -> dict[str, tuple[float, float]]:
        """这只臂实际允许的关节范围（度）。空字典表示只受 URDF 限位约束。"""
        return {}


class Head(ABC):
    """2 自由度云台。角度单位为度，0 表示配置里的 center 刻度。"""

    def __init__(self):
        self.connected = False
        self.error: str | None = None

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def read(self) -> dict[str, float]:
        """返回 {"pan": 度, "tilt": 度}。"""

    @abstractmethod
    def write(self, target: dict[str, float]) -> None: ...

    @abstractmethod
    def limits(self) -> dict[str, tuple[float, float]]: ...
