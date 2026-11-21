"""真机驱动：基于 LeRobot 的 Feetech 总线。

机械臂的校准直接读舵机 EEPROM 里已有的零点偏置和行程限位，不使用校准文件，也不改写
舵机里的校准。角度换算以刻度 2047 为零度：LeRobot 校准时把手臂摆在中间姿态并把该处
设为 2047，这个姿态与 URDF 的零位一致。没有采用 LeRobot 自带的“行程中点为零”的换算，
是因为这套设备上次校准时有的关节没走满行程，行程中点与真实零位能差到 20° 左右。

连接流程自己写而不调用 SOFollower.connect()，是为了控制上力矩的顺序：
关力矩 → 写配置 → 目标对齐到当前位置 → 上力矩，保证连接瞬间手臂不跳动。
"""

from __future__ import annotations

import logging

from lerobot.motors import Motor, MotorNormMode, SerialMotorsBus

from ..config import ArmConfig, HeadConfig, SafetyConfig
from .base import JOINTS, Arm, Head

logger = logging.getLogger(__name__)

_ARM_BODY = JOINTS[:-1]
_TICKS_PER_DEG = 4096 / 360.0
_ZERO_TICK = 2047


def make_arm_bus(port: str) -> SerialMotorsBus:
    """SO-100/101 follower 的 6 个 STS3215，ID 1-6。读写都用原始刻度。"""
    return SerialMotorsBus(
        port=port,
        motors={name: Motor(i + 1, "sts3215", MotorNormMode.DEGREES) for i, name in enumerate(JOINTS)},
    )


class RealArm(Arm):
    def __init__(self, name: str, cfg: ArmConfig, safety: SafetyConfig):
        super().__init__(name)
        self.cfg = cfg
        self.safety = safety
        self._bus: SerialMotorsBus | None = None
        self._range: dict[str, tuple[int, int]] = {}  # 各关节的行程限位（刻度）

    def _require_bus(self) -> SerialMotorsBus:
        if self._bus is None:
            raise RuntimeError(f"{self.name} 未连接")
        return self._bus

    def open(self) -> None:
        """打开总线并读取舵机内的校准，不改变任何舵机状态。"""
        if not self.cfg.port:
            raise RuntimeError("未配置串口 port，请先运行 soarm-tools scan 并填写 config/config.yaml")
        bus = make_arm_bus(self.cfg.port)
        bus.connect()
        try:
            calib = bus.read_calibration()
            ranges = {j: (c.range_min, c.range_max) for j, c in calib.items()}
            bad = [j for j in JOINTS if j != "wrist_roll" and not 0 < ranges[j][1] - ranges[j][0] < 4000]
            if bad:
                raise RuntimeError(f"关节 {bad} 的行程限位是出厂值，请先运行 soarm-tools calibrate {self.name}")
            self.found_resting = not any(
                bus.sync_read("Torque_Enable", normalize=False, num_retry=3).values()
            )
        except Exception:
            bus.disconnect(disable_torque=False)
            raise
        self._bus, self._range = bus, ranges

    def connect(self) -> None:
        self.open()
        bus = self._require_bus()
        try:
            present = bus.sync_read("Present_Position", normalize=False, num_retry=3)
            if self.found_resting:
                # 手臂本来就无力停放着：显式关力矩以打开 EEPROM 锁，再写入运行参数
                bus.disable_torque(num_retry=3)
                bus.configure_motors(acceleration=self.safety.acceleration)
                for motor in bus.motors:
                    bus.set_operating_mode("position", motor)
                    bus.write("P_Coefficient", motor, 16)
                    bus.write("I_Coefficient", motor, 0)
                    bus.write("D_Coefficient", motor, 32)
            for motor in bus.motors:
                limit = min(self.safety.max_torque_limit, 500) if motor == "gripper" else self.safety.max_torque_limit
                bus.write("Torque_Limit", motor, limit)
            # 先把目标对齐到当前位置再上力矩，避免跳动
            bus.sync_write("Goal_Position", present, normalize=False)
            bus.enable_torque(num_retry=3)
        except Exception:
            bus.disconnect(disable_torque=False)
            self._bus = None
            raise
        self.connected = True
        self.torque_enabled = True
        logger.info("机械臂 %s 已连接：%s", self.name, self.cfg.port)

    def disconnect(self) -> None:
        if self._bus is not None and self._bus.is_connected:
            # 是否释放力矩由控制层决定（急停时保持力矩），这里不改变力矩状态
            self._bus.disconnect(disable_torque=False)
        self._bus = None
        self.connected = False

    # ---------- 刻度与角度换算 ----------
    def _to_units(self, joint: str, ticks: int) -> float:
        if joint == "gripper":
            lo, hi = self._range[joint]
            return (ticks - lo) / (hi - lo) * 100.0
        return (ticks - _ZERO_TICK) / _TICKS_PER_DEG

    def _to_ticks(self, joint: str, value: float) -> int:
        lo, hi = self._range[joint]
        if joint == "gripper":
            ticks = lo + value / 100.0 * (hi - lo)
        else:
            ticks = _ZERO_TICK + value * _TICKS_PER_DEG
        return int(max(lo, min(hi, round(ticks))))

    def limits(self) -> dict[str, tuple[float, float]]:
        return {j: (self._to_units(j, self._range[j][0]), self._to_units(j, self._range[j][1])) for j in _ARM_BODY}

    def read(self) -> dict[str, float]:
        raw = self._require_bus().sync_read("Present_Position", normalize=False, num_retry=2)
        return {j: self._to_units(j, raw[j]) for j in JOINTS}

    def write(self, target: dict[str, float]) -> None:
        raw = {j: self._to_ticks(j, target[j]) for j in JOINTS}
        self._require_bus().sync_write("Goal_Position", raw, normalize=False)

    def set_torque(self, enabled: bool) -> None:
        bus = self._require_bus()
        if enabled:
            present = bus.sync_read("Present_Position", normalize=False, num_retry=3)
            bus.sync_write("Goal_Position", present, normalize=False)
            bus.enable_torque(num_retry=3)
        else:
            bus.disable_torque(num_retry=3)
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
