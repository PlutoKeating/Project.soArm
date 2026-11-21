from __future__ import annotations

from ..config import Config
from .base import Arm, Head


def make_robot(cfg: Config) -> tuple[dict[str, Arm], Head | None]:
    """按配置创建机械臂与云台对象（尚未连接）。mock 模式下不导入 LeRobot。"""
    if cfg.mock:
        from .mock import MockArm, MockHead

        return {name: MockArm(name) for name in cfg.arms}, MockHead()

    from .real import RealArm, RealHead

    arms: dict[str, Arm] = {name: RealArm(name, arm_cfg, cfg.safety) for name, arm_cfg in cfg.arms.items()}
    head = RealHead(cfg.head, cfg.safety) if cfg.head.port else None
    return arms, head
