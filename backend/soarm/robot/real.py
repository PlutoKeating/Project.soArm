"""真机驱动：基于 LeRobot 的 Feetech 总线与 SO follower 校准文件。

机械臂这里没有直接调用 LeRobot 的 SOFollower.connect()，原因是它会在配置电机后直接
恢复力矩，而展会场景需要我们自己控制上力矩的顺序：先关力矩写配置、把目标位置对齐到
当前位置、再上力矩，保证连接瞬间手臂不会跳动。校准文件的读取和格式仍完全沿用 LeRobot，
所以 `lerobot-calibrate` 生成的校准可以直接用。
"""

from __future__ import annotations

import logging

from lerobot.motors import Motor, MotorNormMode, SerialMotorsBus
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from ..config import ArmConfig, HeadConfig, SafetyConfig
from .base import JOINTS, Arm, Head

logger = logging.getLogger(__name__)

_ARM_BODY = JOINTS[:-1]
_TICKS_PER_DEG = 4096 / 360.0


class RealArm(Arm):
    def __init__(self, name: str, cfg: ArmConfig, safety: SafetyConfig):
        super().__init__(name)
        self.cfg = cfg
        self.safety = safety
        self._robot: SO101Follower | None = None

    @property
    def _bus(self) -> SerialMotorsBus:
        if self._robot is None:
            raise RuntimeError(f"{self.name} 未连接")
        return self._robot.bus

    def connect(self) -> None:
        if not self.cfg.port:
            raise RuntimeError("未配置串口 port，请先运行 soarm-tools scan 并填写 config/config.yaml")
        robot = SO101Follower(
            SO101FollowerConfig(port=self.cfg.port, id=self.cfg.calibration_id, use_degrees=True)
        )
        if not robot.calibration:
            raise RuntimeError(
                f"找不到校准文件 {robot.calibration_fpath}，"
                f"请先运行 soarm-tools recover-calibration {self.name} 或 soarm-tools calibrate {self.name}"
            )
        bus = robot.bus
        bus.connect()
        try:
            bus.disable_torque(num_retry=3)
            if not bus.is_calibrated:
                logger.info("%s: 电机内的校准与文件不一致，以文件为准写入电机", self.name)
                bus.write_calibration(robot.calibration)
            bus.configure_motors(acceleration=self.safety.acceleration)
            for motor in bus.motors:
                bus.set_operating_mode("position", motor)
                bus.write("P_Coefficient", motor, 16)
                bus.write("I_Coefficient", motor, 0)
                bus.write("D_Coefficient", motor, 32)
                bus.write("Torque_Limit", motor, self.safety.max_torque_limit)
            # 夹爪堵转保护，数值沿用 LeRobot 的 SO follower 设置
            bus.write("Max_Torque_Limit", "gripper", 500)
            bus.write("Protection_Current", "gripper", 250)
            bus.write("Overload_Torque", "gripper", 25)
            # 先把目标对齐到当前位置再上力矩，避免跳动
            present = bus.sync_read("Present_Position", num_retry=3)
            bus.sync_write("Goal_Position", present)
            bus.enable_torque(num_retry=3)
        except Exception:
            bus.disconnect(disable_torque=False)
            raise
        self._robot = robot
        self.connected = True
        self.torque_enabled = True
        logger.info("机械臂 %s 已连接：%s", self.name, self.cfg.port)

    def disconnect(self) -> None:
        if self._robot is not None and self._robot.bus.is_connected:
            # 是否释放力矩由控制层决定（急停时保持力矩），这里不改变力矩状态
            self._robot.bus.disconnect(disable_torque=False)
        self._robot = None
        self.connected = False

    def read(self) -> dict[str, float]:
        values = self._bus.sync_read("Present_Position", num_retry=2)
        return {j: float(values[j]) for j in JOINTS}

    def write(self, target: dict[str, float]) -> None:
        self._bus.sync_write("Goal_Position", {j: float(target[j]) for j in JOINTS})

    def set_torque(self, enabled: bool) -> None:
        if enabled:
            present = self._bus.sync_read("Present_Position", num_retry=3)
            self._bus.sync_write("Goal_Position", present)
            self._bus.enable_torque(num_retry=3)
        else:
            self._bus.disable_torque(num_retry=3)
        self.torque_enabled = enabled


class RealHead(Head):
    """云台两个 STS3215。不走 LeRobot 的校准流程，直接用原始刻度 + 配置里的中心与限位。"""

    def __init__(self, cfg: HeadConfig, safety: SafetyConfig):
        super().__init__()
        self.cfg = cfg
        self.safety = safety
        self._axes = {"pan": cfg.pan, "tilt": cfg.tilt}
        self._bus: SerialMotorsBus | None = None

    def connect(self) -> None:
        if not self.cfg.port:
            raise RuntimeError("未配置云台串口 port")
        bus = SerialMotorsBus(
            port=self.cfg.port,
            motors={axis: Motor(m.id, "sts3215", MotorNormMode.DEGREES) for axis, m in self._axes.items()},
        )
        bus.connect()
        try:
            bus.disable_torque(num_retry=3)
            bus.configure_motors(acceleration=self.safety.acceleration)
            for axis in self._axes:
                bus.set_operating_mode("position", axis)
                bus.write("Torque_Limit", axis, self.safety.max_torque_limit)
            present = bus.sync_read("Present_Position", normalize=False, num_retry=3)
            bus.sync_write("Goal_Position", present, normalize=False)
            bus.enable_torque(num_retry=3)
        except Exception:
            bus.disconnect(disable_torque=False)
            raise
        self._bus = bus
        self.connected = True
        logger.info("云台已连接：%s", self.cfg.port)

    def disconnect(self) -> None:
        if self._bus is not None and self._bus.is_connected:
            self._bus.disconnect(disable_torque=True)
        self._bus = None
        self.connected = False

    def read(self) -> dict[str, float]:
        raw = self._bus.sync_read("Present_Position", normalize=False, num_retry=2)
        return {axis: (raw[axis] - m.center) / _TICKS_PER_DEG for axis, m in self._axes.items()}

    def write(self, target: dict[str, float]) -> None:
        raw = {}
        for axis, m in self._axes.items():
            ticks = int(round(m.center + target[axis] * _TICKS_PER_DEG))
            raw[axis] = max(m.min, min(m.max, ticks))
        self._bus.sync_write("Goal_Position", raw, normalize=False)

    def limits(self) -> dict[str, tuple[float, float]]:
        return {
            axis: ((m.min - m.center) / _TICKS_PER_DEG, (m.max - m.center) / _TICKS_PER_DEG)
            for axis, m in self._axes.items()
        }
