"""控制核心：一个固定频率的循环，按当前模式给每只臂算出目标关节角并下发。

模式：
  idle      保持当前位置
  keyboard  第 1 步：键盘末端笛卡尔控制（按住才动）
  follow    第 2 步：腕部相机对准跟随（只转动、不调距离）
  fused     第 3 步：深度相机全局引导 + 腕部相机精调

所有模式的输出都要经过同一套安全处理：急停、阻挡让位、关节限速、关节限位。
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from ..calib import (
    ArmExtrinsic,
    BearingSample,
    Calibration,
    CalibSession,
    fit_floor_normal,
    refine_from_bearings,
    solve_from_bearings,
)
from ..config import ArmConfig, Config
from ..kinematics import ARM_JOINTS, IK_JOINTS, ArmKinematics
from .geometry import aim_offsets, wrist_ray
from ..perception.cameras import CameraWorker
from ..perception.hand import view_quality
from ..robot.base import JOINTS, Arm, Head

logger = logging.getLogger(__name__)

MODES = ("idle", "keyboard", "follow", "fused")

# 按键 -> (轴, 方向)。使用浏览器 KeyboardEvent.code，与键盘布局无关
KEYMAP: dict[str, dict[str, tuple[str, float]]] = {
    "left": {
        "KeyW": ("x", +1), "KeyS": ("x", -1),
        "KeyA": ("y", +1), "KeyD": ("y", -1),
        "KeyR": ("z", +1), "KeyF": ("z", -1),
        "KeyQ": ("pitch", +1), "KeyE": ("pitch", -1),
        "KeyZ": ("roll", +1), "KeyX": ("roll", -1),
        "KeyC": ("grip", -1), "KeyV": ("grip", +1),
    },
    "right": {
        "KeyI": ("x", +1), "KeyK": ("x", -1),
        "KeyJ": ("y", +1), "KeyL": ("y", -1),
        "KeyY": ("z", +1), "KeyH": ("z", -1),
        "KeyU": ("pitch", +1), "KeyO": ("pitch", -1),
        "KeyN": ("roll", +1), "KeyM": ("roll", -1),
        "Comma": ("grip", -1), "Period": ("grip", +1),
    },
}  # fmt: skip
HEAD_KEYMAP: dict[str, tuple[str, float]] = {
    "ArrowLeft": ("pan", +1), "ArrowRight": ("pan", -1),
    "ArrowUp": ("tilt", +1), "ArrowDown": ("tilt", -1),
}  # fmt: skip


@dataclass
class CartTarget:
    pos: np.ndarray
    pitch: float


@dataclass
class ArmRuntime:
    arm: Arm
    cfg: ArmConfig
    pos: dict[str, float] = field(default_factory=dict)  # 实测
    target: dict[str, float] = field(default_factory=dict)  # 已下发的目标
    cart: CartTarget | None = None
    goto: dict[str, float] | None = None  # 正在前往的固定姿态
    goto_name: str = ""
    release_after_goto: bool = False
    pan_off: float = 0.0  # 腕部对准产生的偏转量（度）
    tilt_off: float = 0.0
    filt: np.ndarray | None = None  # 平滑后的画面误差
    lost_since: float | None = None
    guided: dict[str, float] | None = None  # 第 3 步引导得到的关节角（不含腕部精调）
    paused_until: float = 0.0
    stall_since: float | None = None
    status: str = ""
    # 腕部跟随（瞄准式）用到的状态
    hist: deque = field(default_factory=lambda: deque(maxlen=45))  # (时刻, 实测 pan 偏转, 实测 tilt 偏转)
    aim: np.ndarray | None = None  # 手所在方向对应的偏转量 (pan, tilt)，度
    aim_vel: np.ndarray = field(default_factory=lambda: np.zeros(2))  # 上述方向的角速度，度/秒
    aim_t: float = 0.0  # 最近一次用于更新 aim 的观测时刻
    enabled: bool = True  # 停用的臂在任何模式下都只保持当前位置
    lent: bool = False  # 已借给开发程序：本进程松开了串口，舵机力矩保持
    home: dict[str, float] = field(default_factory=dict)  # 本臂的初始位
    limits: dict[str, tuple[float, float]] = field(default_factory=dict)  # 本臂生效的关节限位
    rest: dict[str, float] | None = None  # 本臂的休息位：上电时手臂自然停放的姿态


def _approach(value: float, goal: float, max_step: float) -> float:
    return value + float(np.clip(goal - value, -max_step, max_step))


class Controller:
    def __init__(
        self,
        cfg: Config,
        arms: dict[str, Arm],
        head: Head | None,
        cameras: dict[str, CameraWorker],
    ):
        self.cfg = cfg
        self.kin = ArmKinematics()
        self.arms = {
            name: ArmRuntime(arm, cfg.arms[name], home={**cfg.home, **cfg.arms[name].home})
            for name, arm in arms.items()
        }
        self.head = head
        self.head_target: dict[str, float] | None = None
        self.head_pos: dict[str, float] = {}
        self.cameras = cameras
        self.mode = "idle"
        self.estop = False
        self.message = ""
        self.loop_hz = 0.0
        self._keys: set[str] = set()
        self._keys_time = 0.0
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="control", daemon=True)

        self.calib_path = cfg.path(cfg.calibration_dir) / "extrinsics.yaml"
        self.calibration = Calibration.load(self.calib_path)
        self.calib_session = CalibSession()
        self._fused_arm: str | None = None
        self._fused_point: np.ndarray | None = None
        self._follow_arm: str | None = None  # 腕部跟随模式下当前负责跟随的臂
        self._follow_rival: tuple[str, float] | None = None  # (候选臂, 开始领先的时刻)
        self._follow_lost_at: float | None = None  # 当前跟随臂开始看不到手的时刻
        # 深度引导：每只臂“要对准深度相机看到的手”所需的 (偏转量, 观测时刻, 转角代价)
        self._cues: dict[str, tuple[np.ndarray, float, float]] = {}
        self._cue_arm: str | None = None  # 当前由深度引导带着转的那只臂（至多一只）
        self._floor_up: np.ndarray | None = None  # 此刻地面法线在深度相机系下的方向
        self._floor_time = 0.0
        # 引导效果：深度引导把臂带过去后，腕部相机第一眼看到手时，手离画面中心的偏差
        self.guidance_hits: deque = deque(maxlen=200)
        self._was_guided: dict[str, float] = {}  # 臂名 -> 最近一次处于深度引导的时刻
        # 外参自动标定积累的配对样本
        self.auto_samples: dict[str, list[BearingSample]] = {name: self._load_auto_samples(name) for name in arms}

        self.home = dict(cfg.home)
        # 肩部参考点：shoulder_lift 关节大致所在位置，用作“伸出距离”的原点
        self.shoulder = self._shoulder_point()
        # 腕部对准用到的方向关系，由运动学在初始位处数值求得
        self._pan_dir, self._tilt_dir = self._follow_directions()

    # ------------------------------------------------------------------ 初始化
    def _shoulder_point(self) -> np.ndarray:
        T = np.eye(4)
        for j in self.kin.chain:
            T = T @ j.origin
            if j.name == "shoulder_lift":
                return T[:3, 3].copy()
        return np.array([0.0, 0.0, 0.1])

    def _follow_directions(self) -> tuple[float, float]:
        """+1° shoulder_pan 让夹爪指向的方位角变化的符号；+1° wrist_flex 让俯仰角变化的符号。"""
        base = {j: self.home[j] for j in ARM_JOINTS}
        a0 = self.kin.approach_dir(base)
        a1 = self.kin.approach_dir({**base, "shoulder_pan": base["shoulder_pan"] + 1.0})
        yaw = np.arctan2(a1[1], a1[0]) - np.arctan2(a0[1], a0[0])
        _, p0 = self.kin.tip_and_pitch(base)
        _, p1 = self.kin.tip_and_pitch({**base, "wrist_flex": base["wrist_flex"] + 1.0})
        return float(np.sign(yaw) or 1.0), float(np.sign(p1 - p0) or 1.0)

    def _arm_limits(self, rt: ArmRuntime) -> dict[str, tuple[float, float]]:
        """URDF 限位、这只臂自身行程限位、配置里额外限位三者的交集。"""
        margin = self.cfg.safety.joint_limit_margin_deg
        own = rt.arm.limits()
        out = {}
        for j in ARM_JOINTS:
            lo, hi = self.kin.limits[j]
            lo, hi = lo + margin, hi - margin
            if j in own:
                lo, hi = max(lo, own[j][0] + 2.0), min(hi, own[j][1] - 2.0)
            if j in rt.cfg.joint_limits:
                lo, hi = max(lo, rt.cfg.joint_limits[j][0]), min(hi, rt.cfg.joint_limits[j][1])
            out[j] = (lo, hi)
        return out

    def start(self) -> None:
        self.connect_all()
        self._thread.start()

    def connect_all(self) -> None:
        """连接尚未连上的设备。失败只记录错误，不影响其它设备。"""
        with self._lock:
            for name, rt in self.arms.items():
                if rt.arm.connected or rt.lent:
                    continue
                try:
                    rt.arm.connect()
                    was_resting = rt.rest is None and rt.arm.found_resting
                    rt.pos = rt.arm.read()
                    rt.target = dict(rt.pos)  # 以当前位置为目标，上电不跳变
                    rt.limits = self._arm_limits(rt)
                    if was_resting:
                        # 首次连接时手臂是无力停放的，这个姿态就是它最稳妥的休息位
                        rt.rest = {**rt.pos, "gripper": self.cfg.rest["gripper"]}
                    rt.arm.error = None
                    rt.status = "已连接"
                except Exception as e:
                    rt.arm.error = str(e)
                    rt.arm.connected = False
                    logger.error("机械臂 %s 连接失败: %s", name, e)
            if self.head is not None and not self.head.connected:
                try:
                    self.head.connect()
                    self.head_pos = self.head.read()
                    self.head_target = dict(self.head_pos)
                    self.head.error = None
                except Exception as e:
                    self.head.error = str(e)
                    self.head.connected = False
                    logger.error("云台连接失败: %s", e)

    def shutdown(self, park_timeout_s: float = 8.0) -> None:
        """退出：未急停时先收到休息位再释放力矩，避免断电瞬间手臂砸下来。"""
        with self._lock:
            active = [rt for rt in self.arms.values() if rt.arm.connected and rt.arm.torque_enabled]
            if active and not self.estop:
                self.mode = "idle"
                for rt in active:
                    self._start_goto(rt, rt.rest or self.cfg.rest, "rest", release=True)
        deadline = time.monotonic() + park_timeout_s
        while time.monotonic() < deadline and any(rt.goto is not None for rt in self.arms.values()):
            time.sleep(0.05)
        self._stop.set()
        self._thread.join(timeout=2)
        for rt in self.arms.values():
            if rt.arm.connected:
                try:
                    rt.arm.disconnect()
                except Exception as e:
                    logger.warning("断开 %s 失败: %s", rt.arm.name, e)
        if self.head is not None and self.head.connected:
            self.head.disconnect()

    # ------------------------------------------------------------------ 外部命令
    def set_keys(self, keys: list[str]) -> None:
        with self._lock:
            self._keys = set(keys)
            self._keys_time = time.monotonic()

    def set_mode(self, mode: str) -> None:
        if mode not in MODES:
            raise ValueError(f"未知模式 {mode}")
        with self._lock:
            if self.estop:
                raise RuntimeError("急停中，请先解除急停")
            self.mode = mode
            self._fused_arm = None
            self._follow_arm = self._follow_rival = self._follow_lost_at = None
            self._cues = {}
            self._cue_arm = None
            for rt in self.arms.values():
                self._reset_motion(rt)
                if not rt.enabled:
                    continue
                if mode in ("follow", "fused") or (mode == "keyboard" and not self._in_workspace(rt)):
                    self._start_goto(rt, rt.home, "home")
            if mode == "fused" and self.calibration.head_preset and self.head_target is not None:
                self.head_target = dict(self.calibration.head_preset)
            self.message = ""

    def set_estop(self, active: bool) -> None:
        with self._lock:
            self.estop = active
            self.mode = "idle"
            self._keys.clear()
            for rt in self.arms.values():
                self._reset_motion(rt)
                rt.goto = None
                if rt.pos:
                    rt.target = dict(rt.pos)  # 冻结在当前位置
                rt.status = "急停" if active else "已解除急停"

    def go(self, pose: str, release: bool = False) -> None:
        """回初始位 (home) 或休息位 (rest)。release=True 时到位后释放力矩。"""
        if pose not in ("home", "rest"):
            raise ValueError(f"未知姿态 {pose}")
        with self._lock:
            if self.estop:
                raise RuntimeError("急停中，请先解除急停")
            self.mode = "idle"
            for rt in self.arms.values():
                self._reset_motion(rt)
                if not rt.enabled:
                    continue
                target = rt.home if pose == "home" else (rt.rest or self.cfg.rest)
                self._start_goto(rt, target, pose, release=release)

    def goto_pose(self, arm: str, pose: dict[str, float], name: str = "pose") -> None:
        """让一只臂以回位速度前往给定的关节姿态（会按本臂限位裁剪）。标定等工具使用。"""
        with self._lock:
            if self.estop:
                raise RuntimeError("急停中，请先解除急停")
            rt = self.arms[arm]
            self._reset_motion(rt)
            self._start_goto(rt, {**rt.target, **pose}, name)

    def is_moving(self, arm: str) -> bool:
        with self._lock:
            return self.arms[arm].goto is not None

    def set_arm_enabled(self, arm: str, enabled: bool) -> None:
        """停用或启用一只臂。停用后它在任何模式下都只保持当前位置。"""
        with self._lock:
            rt = self.arms[arm]
            rt.enabled = enabled
            self._reset_motion(rt)
            rt.goto = None
            if enabled and self.mode in ("follow", "fused") and not self.estop:
                self._start_goto(rt, rt.home, "home")

    def lend_arm(self, arm: str) -> None:
        """把一只臂借给开发程序：停止指挥它并松开串口，舵机力矩不变、臂停在原位。"""
        with self._lock:
            rt = self.arms[arm]
            if rt.lent:
                return
            self._reset_motion(rt)
            rt.goto = None
            if rt.arm.connected:
                rt.arm.disconnect()
            rt.arm.connected = False
            rt.lent = True
            rt.arm.error = "已借给开发调试"
            rt.status = "已借出"
            logger.info("机械臂 %s 已借给开发程序", arm)

    def restore_all(self) -> dict:
        """恢复全量展示：让借用机械臂的开发程序退出，把借出的臂全部接管回来。"""
        from .. import lab

        note = lab.evict()
        with self._lock:
            lent = [name for name, rt in self.arms.items() if rt.lent]
            for name in lent:
                self.arms[name].lent = False
        # 开发程序刚松开串口，系统可能还要一瞬间才放行，重试几次
        for _ in range(12):
            self.connect_all()
            if all(self.arms[name].arm.connected for name in lent):
                break
            time.sleep(0.25)
        with self._lock:
            for name in lent:
                rt = self.arms[name]
                if rt.arm.connected and self._pointing_mode_or_follow() and not self.estop:
                    self._start_goto(rt, rt.home, "home")
            failed = [name for name in lent if not self.arms[name].arm.connected]
        logger.info("恢复全量展示：%s；接管 %s；失败 %s", note, lent, failed)
        return {"evicted": note, "restored": [n for n in lent if n not in failed], "failed": failed}

    def _pointing_mode_or_follow(self) -> bool:
        return self.mode in ("follow", "fused")

    def abort(self) -> None:
        """立刻停手并松开所有串口，不收手、不动力矩。给开发程序被要求让路时用。"""
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1)
        for rt in self.arms.values():
            if rt.arm.connected:
                try:
                    rt.arm.disconnect()
                except Exception as e:
                    logger.warning("松开 %s 失败: %s", rt.arm.name, e)

    def set_torque(self, enabled: bool) -> None:
        with self._lock:
            for rt in self.arms.values():
                if not rt.arm.connected:
                    continue
                if enabled and rt.pos:
                    rt.target = dict(rt.pos)
                rt.arm.set_torque(enabled)
                self._reset_motion(rt)
                rt.goto = None
            if not enabled:
                self.mode = "idle"

    def _reset_motion(self, rt: ArmRuntime) -> None:
        rt.cart = None
        rt.pan_off = rt.tilt_off = 0.0
        rt.filt = None
        rt.lost_since = None
        rt.guided = None
        rt.status = ""
        rt.aim = None
        rt.aim_vel = np.zeros(2)
        rt.aim_t = 0.0
        rt.hist.clear()

    def _start_goto(self, rt: ArmRuntime, pose: dict[str, float], name: str, release: bool = False) -> None:
        if not rt.arm.connected or not rt.arm.torque_enabled:
            return
        # 目标先按本臂限位裁剪，否则够不到的目标会让“到位”永远不成立
        rt.goto = {**pose, **self.kin.clamp({j: pose[j] for j in ARM_JOINTS}, rt.limits)}
        rt.goto_name = name
        rt.release_after_goto = release

    # ------------------------------------------------------------------ 标定
    def calib_add(self, arm: str, u: float, v: float) -> dict:
        with self._lock:
            rt = self.arms[arm]
            if not rt.arm.connected:
                raise RuntimeError(f"{arm} 未连接")
            cam = self.cameras.get(self.cfg.head_camera)
            point = cam.point_at(u, v) if cam else None
            if point is None:
                raise RuntimeError("该像素没有有效深度，请点在夹爪尖上并确认深度相机在线")
            if self.calib_session.count("left") + self.calib_session.count("right") == 0:
                self._calib_head = dict(self.head_pos) if self.head_pos else None
            tip, _ = self.kin.tip_and_pitch({j: rt.pos[j] for j in ARM_JOINTS})
            n = self.calib_session.add(arm, point, tip)
            return {"arm": arm, "count": n, "cam": point.tolist(), "base": tip.tolist()}

    _calib_head: dict[str, float] | None = None

    def calib_undo(self, arm: str) -> int:
        with self._lock:
            return self.calib_session.undo(arm)

    def calib_clear(self, arm: str) -> None:
        with self._lock:
            self.calib_session.clear(arm)

    def calib_solve(self, arm: str) -> dict:
        with self._lock:
            result = self.calib_session.solve(arm)
            self.calibration.arms[arm] = result
            self.calibration.head_preset = self._calib_head or (dict(self.head_pos) if self.head_pos else None)
            self.calibration.save(self.calib_path)
            self.calib_session.clear(arm)
            return {"arm": arm, "rms": result.rms, "samples": result.samples}

    # ------------------------------------------------------------------ 主循环
    def _run(self) -> None:
        period = 1.0 / self.cfg.control_hz
        prev = time.monotonic()
        while not self._stop.is_set():
            start = time.monotonic()
            dt = min(start - prev, 0.1)
            prev = start
            if dt > 0:
                self.loop_hz = 0.9 * self.loop_hz + 0.1 / dt
            with self._lock:
                try:
                    self._tick(dt, start)
                except Exception:
                    logger.exception("控制循环异常，已切到 idle")
                    self.mode = "idle"
            self._stop.wait(max(0.0, period - (time.monotonic() - start)))

    def _active_keys(self, now: float) -> set[str]:
        """按键心跳超时（浏览器失焦、断连、卡死）就当作全部松开。"""
        if now - self._keys_time > self.cfg.safety.key_timeout_s:
            self._keys = set()
        return self._keys

    def _tick(self, dt: float, now: float) -> None:
        keys = self._active_keys(now)
        if self.mode == "fused" and self.cfg.fused.reach and not self.estop:
            self._fused_select()
        if self._pointing_mode() and not self.estop:
            self._depth_tick(now)
            self._follow_select(now)
        for name, rt in self.arms.items():
            if not rt.arm.connected:
                continue
            try:
                self._tick_arm(name, rt, keys, dt, now)
            except Exception as e:
                rt.arm.error = f"通信失败: {e}"
                rt.arm.connected = False
                logger.error("机械臂 %s 通信失败，已标记为断开: %s", name, e)
        self._tick_head(keys, dt)
        self._update_overlays()

    def _tick_arm(self, name: str, rt: ArmRuntime, keys: set[str], dt: float, now: float) -> None:
        rt.pos = rt.arm.read()
        if not rt.target:
            rt.target = dict(rt.pos)
        if not rt.arm.torque_enabled:
            rt.target = dict(rt.pos)
            rt.cart = None
            rt.status = "力矩已释放"
            return
        if self.estop:
            rt.arm.write(rt.target)
            return

        safety = self.cfg.safety
        # 阻挡让位：实测与目标偏差过大，说明有人或物体挡住了，放弃当前目标
        worst = max(abs(rt.pos[j] - rt.target[j]) for j in ARM_JOINTS)
        if worst <= safety.stall_error_deg:
            rt.stall_since = None
        elif rt.stall_since is None:
            rt.stall_since = now  # 快速运动时会有瞬时滞后，偏差要持续一段时间才算受阻
        if rt.stall_since is not None and now - rt.stall_since >= safety.stall_hold_s:
            rt.stall_since = None
            rt.target.update({j: rt.pos[j] for j in ARM_JOINTS})
            self._reset_motion(rt)
            rt.goto = None
            rt.paused_until = now + 1.5
            rt.status = "受到阻挡，已让位"
            logger.warning("机械臂 %s 受到阻挡（偏差 %.0f°），已让位", name, worst)
        if now < rt.paused_until:
            rt.arm.write(rt.target)
            return

        speed = safety.max_joint_speed_deg_s
        if not rt.enabled and rt.goto is None:
            desired = dict(rt.target)
            rt.status = "已停用，保持"
        elif rt.goto is not None:
            desired, speed = dict(rt.goto), safety.home_speed_deg_s
            rt.status = {"home": "回初始位", "rest": "收到休息位"}.get(rt.goto_name, "前往指定姿态")
        elif self.mode == "keyboard":
            desired = self._keyboard(name, rt, keys, dt)
        elif self.mode == "follow":
            desired = self._follow(rt, dt, now)
        elif self.mode == "fused":
            desired = self._fused(name, rt, dt, now) if self.cfg.fused.reach else self._follow(rt, dt, now)
        else:
            desired = dict(rt.target)
            rt.status = rt.status or "保持"

        max_step = speed * dt
        new = {}
        for j in JOINTS:
            step = max_step * (2.5 if j == "gripper" else 1.0)
            new[j] = _approach(rt.target[j], desired.get(j, rt.target[j]), step)
        arm_part = self.kin.clamp({j: new[j] for j in ARM_JOINTS}, rt.limits)
        new.update(arm_part)
        new["gripper"] = float(np.clip(new["gripper"], 0.0, 100.0))
        rt.target = new
        rt.arm.write(new)

        if rt.goto is not None and all(abs(new[j] - rt.goto[j]) < 0.5 for j in JOINTS):
            release, rt.goto = rt.release_after_goto, None
            rt.status = "已到位"
            if release:
                rt.arm.set_torque(False)

    # ------------------------------------------------------------------ 工作空间
    def _clamp_workspace(self, rt: ArmRuntime, pos: np.ndarray) -> np.ndarray:
        s = self.cfg.safety
        out = np.array(
            [np.clip(pos[0], *s.x_range), np.clip(pos[1], *s.y_range), np.clip(pos[2], *s.z_range)]
        )
        # 不越过两臂中线
        inward = out[1] * rt.cfg.inward_y_sign
        if inward > rt.cfg.inward_limit:
            out[1] = rt.cfg.inward_limit * rt.cfg.inward_y_sign
        return out

    def _in_workspace(self, rt: ArmRuntime) -> bool:
        if not rt.target:
            return False
        tip, _ = self.kin.tip_and_pitch({j: rt.target[j] for j in ARM_JOINTS})
        return bool(np.allclose(tip, self._clamp_workspace(rt, tip), atol=1e-6))

    def _solve(self, rt: ArmRuntime, pos: np.ndarray, pitch: float, seed: dict[str, float]):
        res = self.kin.ik(pos, pitch, {j: seed[j] for j in ARM_JOINTS}, limits=rt.limits)
        ok = res.pos_error <= self.cfg.safety.ik_pos_tolerance and res.pitch_error <= 4.0
        return res if ok else None

    # ------------------------------------------------------------------ 第 1 步：键盘
    def _keyboard(self, name: str, rt: ArmRuntime, keys: set[str], dt: float) -> dict[str, float]:
        kb, s = self.cfg.keyboard, self.cfg.safety
        desired = dict(rt.target)
        axes = {"x": 0.0, "y": 0.0, "z": 0.0, "pitch": 0.0, "roll": 0.0, "grip": 0.0}
        for code, (axis, sign) in KEYMAP.get(name, {}).items():
            if code in keys:
                axes[axis] += sign
        moving = any(axes[a] for a in ("x", "y", "z", "pitch"))
        if not moving:
            rt.cart = None  # 松开即停：下次按键从当时的目标重新出发
            rt.status = "待命"
        else:
            if rt.cart is None:
                tip, pitch = self.kin.tip_and_pitch({j: rt.target[j] for j in ARM_JOINTS})
                rt.cart = CartTarget(tip, pitch)
            delta = np.array([axes["x"], axes["y"], axes["z"]]) * kb.linear_speed * dt
            cand_pos = self._clamp_workspace(rt, rt.cart.pos + delta)
            cand_pitch = float(
                np.clip(rt.cart.pitch + axes["pitch"] * kb.pitch_speed_deg_s * dt, *s.pitch_range_deg)
            )
            res = self._solve(rt, cand_pos, cand_pitch, rt.target)
            if res is None:
                rt.status = "到达可达范围边界"
            else:
                rt.cart = CartTarget(cand_pos, cand_pitch)
                desired.update(res.joints)
                rt.status = "移动中"
        if axes["roll"]:
            desired["wrist_roll"] = rt.target["wrist_roll"] + axes["roll"] * kb.roll_speed_deg_s * dt * 3
        if axes["grip"]:
            desired["gripper"] = rt.target["gripper"] + axes["grip"] * kb.gripper_speed * dt * 3
        return desired

    # ------------------------------------------------------------------ 第 2 步：腕部对准
    def _wrist_error(self, rt: ArmRuntime) -> tuple[float, float] | None:
        """腕部相机里主手相对画面中心的角度误差（度，右/下为正）。看不到手返回 None。"""
        cam = self.cameras.get(rt.cfg.camera)
        f = self.cfg.follow
        hand = cam.primary_hand() if cam else None
        if hand is None or hand.score < f.min_hand_score:
            rt.filt = None
            return None
        e = np.array([(hand.cx - 0.5) * 2.0, (hand.cy - 0.5) * 2.0])
        rt.filt = e if rt.filt is None else f.smoothing * e + (1 - f.smoothing) * rt.filt
        # 死区之外才修正，并减去死区宽度以保证修正量连续
        ex, ey = (float(np.sign(v) * max(0.0, abs(v) - f.deadzone)) for v in rt.filt)
        hfov = cam.cfg.hfov_deg
        aspect = cam.cfg.height / cam.cfg.width if cam.cfg.rotate in (0, 180) else cam.cfg.width / cam.cfg.height
        return ex * hfov / 2.0, ey * hfov * aspect / 2.0

    def _apply_wrist_error(self, rt: ArmRuntime, err: tuple[float, float], dt: float,
                           pan_range: tuple[float, float], tilt_range: tuple[float, float]) -> None:
        f = self.cfg.follow
        rate_x = float(np.clip(f.gain * err[0], -f.max_speed_deg_s, f.max_speed_deg_s))
        rate_y = float(np.clip(f.gain * err[1], -f.max_speed_deg_s, f.max_speed_deg_s))
        # 手在画面右侧 -> 夹爪向右转（方位角减小）；手在画面下方 -> 夹爪低头（俯仰角减小）
        rt.pan_off += -rate_x * dt * self._pan_dir * rt.cfg.follow_pan_sign
        rt.tilt_off += -rate_y * dt * self._tilt_dir * rt.cfg.follow_tilt_sign
        rt.pan_off = float(np.clip(rt.pan_off, *pan_range))
        rt.tilt_off = float(np.clip(rt.tilt_off, *tilt_range))
        # 再按本臂关节限位收紧，避免偏转量在限位之外继续累积
        base = rt.guided or rt.home
        for attr, joint in (("pan_off", "shoulder_pan"), ("tilt_off", "wrist_flex")):
            lo, hi = rt.limits[joint]
            setattr(rt, attr, float(np.clip(getattr(rt, attr), lo - base[joint], hi - base[joint])))

    def _view_quality(self, rt: ArmRuntime) -> float:
        """这只臂的腕部相机看手看得有多好，0 表示看不到。

        综合三点：检测置信度、手离画面中心的远近、手在画面里的大小。手越居中、越完整、
        检测越稳，这只臂就越适合去跟。
        """
        cam = self.cameras.get(rt.cfg.camera)
        hand = cam.primary_hand() if cam else None
        if hand is None or hand.score < self.cfg.follow.min_hand_score:
            return 0.0
        return view_quality(hand)

    def _follow_select(self, now: float) -> None:
        """两只臂同一时刻至多一只在跟随：选视野最好的那只，带滞回。"""
        f = self.cfg.follow
        if not f.single_tracker:
            self._follow_arm = None
            return
        quality = {
            name: self._view_quality(rt)
            for name, rt in self.arms.items()
            if rt.enabled and rt.arm.connected and rt.arm.torque_enabled and rt.goto is None
        }
        seeing = {n: q for n, q in quality.items() if q > 0}
        current = self._follow_arm
        cues = {n: c for n, c in self._cues.items() if n in quality}
        if not seeing and self._cue_arm in cues:
            # 没有腕部相机看到手，但深度相机看到了：跟随权交给被深度引导选中的那只臂
            self._follow_arm = self._cue_arm
            self._follow_rival, self._follow_lost_at = None, None
            return
        if current is None:
            self._follow_arm = max(seeing, key=seeing.get) if seeing else None
            self._follow_rival, self._follow_lost_at = None, None
            return
        if current not in seeing:
            # 当前跟随臂看不到手了。检测偶尔会漏一两帧，所以先等一小段宽限期，
            # 确实丢了再移交给别的臂（或放弃跟随权）
            if self._follow_lost_at is None:
                self._follow_lost_at = now
            if now - self._follow_lost_at >= f.switch_hold_s or current not in quality:
                self._follow_arm = max(seeing, key=seeing.get) if seeing else None
                self._follow_rival, self._follow_lost_at = None, None
            return
        self._follow_lost_at = None
        best = max(seeing, key=seeing.get)
        if best == current or seeing[best] < seeing[current] * (1 + f.switch_margin):
            self._follow_rival = None
            return
        # 另一只臂明显更好：要持续领先一小段时间才换，避免抖动
        if self._follow_rival is None or self._follow_rival[0] != best:
            self._follow_rival = (best, now)
        elif now - self._follow_rival[1] >= f.switch_hold_s:
            self._follow_arm, self._follow_rival = best, None

    def _offset_limits(self, rt: ArmRuntime) -> tuple[np.ndarray, np.ndarray]:
        """腕部跟随的偏转量允许范围：配置范围与本臂关节限位的交集。"""
        f = self.cfg.follow
        lo, hi = [], []
        for joint, (rlo, rhi) in (("shoulder_pan", f.pan_range_deg), ("wrist_flex", f.tilt_range_deg)):
            jlo, jhi = rt.limits[joint]
            lo.append(max(rlo, jlo - rt.home[joint]))
            hi.append(min(rhi, jhi - rt.home[joint]))
        return np.array(lo), np.array(hi)

    def _measured_offset_at(self, rt: ArmRuntime, t: float) -> np.ndarray:
        """t 时刻手臂实际的偏转量。观测有延迟，要用拍下那一帧时手臂所在的位置来换算。"""
        best = min(rt.hist, key=lambda h: abs(h[0] - t))
        return np.array([best[1], best[2]])

    def _pointing_mode(self) -> bool:
        """当前是否在用“转动对准”的方式跟随：腕部跟随，或指向式的深度引导。"""
        return self.mode == "follow" or (self.mode == "fused" and not self.cfg.fused.reach)

    @staticmethod
    def _aspect(cam) -> float:
        return cam.cfg.height / cam.cfg.width if cam.cfg.rotate in (0, 180) else cam.cfg.width / cam.cfg.height

    def _depth_tick(self, now: float) -> None:
        """处理深度相机这一拍的观测：算出各臂的引导量，并为外参自动标定积累样本。"""
        self._cues = {}
        fc = self.cfg.fused
        cam = self.cameras.get(self.cfg.head_camera)
        hand = cam.primary_hand(max_age_s=fc.cue_max_age_s) if cam else None
        if hand is None or hand.xyz is None or not (fc.depth_range[0] <= hand.xyz[2] <= fc.depth_range[1]):
            return

        if self.mode == "fused" and self._head_matches_preset():
            self._refresh_floor(cam, now)
            for name, rt in self.arms.items():
                ext = self.calibration.arms.get(name)
                if ext is None or not rt.limits:
                    continue
                T = ext.at(self._floor_up)
                point = T[:3, :3] @ hand.xyz + T[:3, 3]
                home = {j: rt.home[j] for j in ARM_JOINTS}
                offsets = aim_offsets(self.kin, home, point, self._pan_dir, self._tilt_dir)
                lo, hi = self._offset_limits(rt)
                # 超出可转范围太多就够不着：留 20° 余量，因为腕部相机视野很宽
                if np.all(offsets >= lo - 20.0) and np.all(offsets <= hi + 20.0):
                    self._cues[name] = (offsets, hand.t, float(abs(offsets[0]) + 0.5 * abs(offsets[1])))

        self._cue_arm = self._pick_cue_arm()

        # 深度相机画面里恰好只有一只手时才采样：有多只时无法确定哪只是腕部相机在跟的
        if fc.auto_calibrate and cam.hand_count() == 1:
            for name in self.arms:
                if not self.cfg.follow.single_tracker or name == self._follow_arm:
                    self._collect_auto_sample(name, hand)

    def _refresh_floor(self, cam, now: float) -> None:
        """每隔几秒从深度图重新测一次地面法线，用来把外参换算到云台当前的俯仰下。"""
        if now - self._floor_time < 5.0 and self._floor_up is not None:
            return
        self._floor_time = now
        latest = cam.latest() if hasattr(cam, "latest") else None
        if latest is None or latest[1] is None or cam.intrinsics is None:
            return
        intr = cam.intrinsics
        # 用标定时记录的“上”方向作先验：相机抬得较平时墙面比地面大，没有先验会认错。
        # 找不到相符的地面（被人挡住等）就沿用上一次的；从没找到过则直接用标定值
        reference = next((e.up_cam for e in self.calibration.arms.values() if e.up_cam is not None), None)
        try:
            up, _ = fit_floor_normal(latest[1], (intr.fx, intr.fy, intr.cx, intr.cy), expected_up=reference)
        except ValueError:
            return
        self._floor_up = up

    def _pick_cue_arm(self) -> str | None:
        """深度引导交给哪只臂：只在自己腕部相机看不到手的臂里选，转角最省的优先。

        同一只手只带一只臂过去；当前被引导的臂只要差得不多就继续由它来，避免来回换。
        单臂跟随模式下，只要有任何腕部相机看到手就不引导。
        """
        blind = {}
        any_seeing = False
        for name, rt in self.arms.items():
            if not (rt.enabled and rt.arm.connected and rt.arm.torque_enabled and rt.goto is None):
                continue
            if self._view_quality(rt) > 0:
                any_seeing = True
            elif name in self._cues:
                blind[name] = self._cues[name][2]
        if not blind or (self.cfg.follow.single_tracker and any_seeing):
            return None
        best = min(blind, key=blind.get)
        current = self._cue_arm
        if current in blind and blind[current] <= blind[best] + self.cfg.fused.cue_switch_margin_deg:
            return current
        return best

    def _collect_auto_sample(self, name: str, head_hand) -> None:
        """某只臂的腕部相机和深度相机同时看到手时，记下一对“视线 + 3D 点”。"""
        fc = self.cfg.fused
        rt = self.arms[name]
        if not rt.pos or not rt.enabled or rt.goto is not None:
            return
        cam = self.cameras.get(rt.cfg.camera)
        wrist_hand = cam.primary_hand(max_age_s=0.15) if cam else None
        if wrist_hand is None or wrist_hand.score < self.cfg.follow.min_hand_score:
            return
        if abs(wrist_hand.t - head_hand.t) > 0.07:  # 两路观测要足够同时
            return
        if max(abs(wrist_hand.cx - 0.5), abs(wrist_hand.cy - 0.5)) * 2.0 > fc.auto_calib_center:
            return  # 视场角是估计值，只有画面中部换算出的方向才够准
        samples = self.auto_samples[name]
        if samples and np.linalg.norm(samples[-1].cam_point - head_hand.xyz) < fc.auto_calib_spacing:
            return
        origin, direction = wrist_ray(
            self.kin, {j: rt.pos[j] for j in ARM_JOINTS}, wrist_hand.cx, wrist_hand.cy,
            cam.cfg.hfov_deg, self._aspect(cam),
        )
        sample = BearingSample(origin, direction, np.array(head_hand.xyz, float))
        samples.append(sample)
        if len(samples) > fc.auto_calib_max_samples:
            del samples[0]
        self._append_auto_sample(name, sample)

    def _auto_samples_path(self, name: str):
        return self.cfg.path(self.cfg.calibration_dir) / f"auto_samples_{name}.jsonl"

    def _append_auto_sample(self, name: str, sample: BearingSample) -> None:
        """样本落盘：展会上一对样本来之不易，进程重启不能丢。"""
        import json

        try:
            path = self._auto_samples_path(name)
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as f:
                f.write(json.dumps({
                    "origin": sample.origin.tolist(), "direction": sample.direction.tolist(),
                    "cam_point": sample.cam_point.tolist(), "time": time.time(),
                }) + "\n")
        except OSError as e:
            logger.warning("自动标定样本落盘失败: %s", e)

    def _load_auto_samples(self, name: str) -> list[BearingSample]:
        import json

        path = self._auto_samples_path(name)
        if not path.exists():
            return []
        out = []
        for line in path.read_text(encoding="utf-8").splitlines():
            try:
                r = json.loads(line)
                out.append(BearingSample(np.array(r["origin"]), np.array(r["direction"]), np.array(r["cam_point"])))
            except (ValueError, KeyError):
                continue
        return out[-self.cfg.fused.auto_calib_max_samples :]

    def calib_auto_solve(self, arm: str) -> dict:
        """用自动积累的样本求解该臂的外参并保存。"""
        with self._lock:
            samples = self.auto_samples[arm]
            existing = self.calibration.arms.get(arm)
            if existing is not None and existing.up_cam is not None:
                # 已有带地面法线的外参：在它的基础上精修，只动转角和平移
                up = self._floor_up if self._floor_up is not None else existing.up_cam
                T, rms_deg, used = refine_from_bearings(samples, existing.at(up), up)
                moved = float(np.linalg.norm(T[:3, 3] - existing.T_base_cam[:3, 3]))
                self.calibration.arms[arm] = ArmExtrinsic(T, existing.rms, used, up)
                self.calibration.save(self.calib_path)
                return {"arm": arm, "rms_deg": rms_deg, "samples": used, "collected": len(samples),
                        "moved_m": moved, "method": "refine"}
            T, rms, used = solve_from_bearings(samples)
            self.calibration.arms[arm] = ArmExtrinsic(T, rms, used)
            if self.head_pos:
                self.calibration.head_preset = dict(self.head_pos)
            self.calibration.save(self.calib_path)
            return {"arm": arm, "rms": rms, "samples": used, "collected": len(samples), "method": "solve"}

    def calib_auto_clear(self, arm: str) -> None:
        with self._lock:
            self.auto_samples[arm] = []
            self._auto_samples_path(arm).unlink(missing_ok=True)

    def _follow(self, rt: ArmRuntime, dt: float, now: float) -> dict[str, float]:
        """瞄准式跟随：每来一帧观测，就算出“手在哪个方向”，直接把目标对准那里。

        与逐拍累加误差相比，这样同一帧观测不会被重复计入，延迟只剩检测和舵机本身；
        再用手的角速度向前预判一点，并在手出画面后沿原方向继续追一小段。
        """
        f = self.cfg.follow
        rt.hist.append((now, rt.pos["shoulder_pan"] - rt.home["shoulder_pan"],
                        rt.pos["wrist_flex"] - rt.home["wrist_flex"]))
        yielding = f.single_tracker and self._pointing_mode() and self._follow_arm not in (None, rt.arm.name)
        cam = self.cameras.get(rt.cfg.camera)
        hand = None if yielding or cam is None else cam.primary_hand(max_age_s=0.25)
        if hand is not None and hand.score < f.min_hand_score:
            hand = None
        lo, hi = self._offset_limits(rt)
        current = np.array([rt.pan_off, rt.tilt_off])

        # 观测来源：优先用自己的腕部相机；它看不到时，用深度相机给的引导
        observed, observed_t, guided = None, 0.0, False
        if hand is not None:
            if hand.t > rt.aim_t:
                # 像素误差 -> 角度误差 -> 手所在方向对应的偏转量
                e = np.array([(hand.cx - 0.5) * 2.0, (hand.cy - 0.5) * 2.0])
                e = np.sign(e) * np.maximum(0.0, np.abs(e) - f.deadzone)
                hfov = cam.cfg.hfov_deg
                err_deg = e * np.array([hfov / 2.0, hfov * self._aspect(cam) / 2.0]) * f.aim_scale
                # 手在画面右侧 -> 向右转（方位角减小）；手在画面下方 -> 低头（俯仰角减小）
                signs = np.array([self._pan_dir * rt.cfg.follow_pan_sign, self._tilt_dir * rt.cfg.follow_tilt_sign])
                observed = np.clip(self._measured_offset_at(rt, hand.t) - err_deg * signs, lo, hi)
                observed_t = hand.t
        elif not yielding and self._cue_arm == rt.arm.name:
            cue = self._cues.get(rt.arm.name)
            if cue is not None and now - cue[1] <= self.cfg.fused.cue_max_age_s:
                guided = True
                if cue[1] > rt.aim_t:
                    observed, observed_t = np.clip(cue[0], lo, hi), cue[1]

        if observed is not None:
            if rt.aim is None:
                rt.aim, rt.aim_vel = observed, np.zeros(2)
            else:
                span = max(observed_t - rt.aim_t, 1e-3)
                blended = rt.aim + f.smoothing * (observed - rt.aim)
                vel = np.clip((blended - rt.aim) / span, -f.max_speed_deg_s, f.max_speed_deg_s)
                rt.aim_vel = 0.5 * rt.aim_vel + 0.5 * vel
                rt.aim = blended
            rt.aim_t = observed_t
            rt.lost_since = None

        name = rt.arm.name
        if guided:
            self._was_guided[name] = now
        elif hand is not None and now - self._was_guided.pop(name, -1e9) < 0.6:
            # 刚从深度引导切到腕部相机：记下手在腕部画面里离中心多远，这就是引导的误差
            self.guidance_hits.append(
                {"arm": name, "time": time.time(), "cx": round(hand.cx, 3), "cy": round(hand.cy, 3)}
            )
            logger.info("深度引导命中 %s：腕部画面里手在 (%.2f, %.2f)", name, hand.cx, hand.cy)

        if (hand is not None or guided) and rt.aim is not None:
            goal = rt.aim + rt.aim_vel * (now - rt.aim_t + f.lead_s)
            on_target = float(np.max(np.abs(goal - current))) < 1.0
            if guided:
                rt.status = "深度引导·已对准" if on_target else "深度引导"
            else:
                rt.status = "已对准" if on_target else "跟随中"
        elif yielding:
            goal, rt.aim = np.zeros(2), None
            rt.status = "让位·另一只臂在跟随"
        else:
            rt.lost_since = rt.lost_since or now
            lost_for = now - rt.lost_since
            if rt.aim is not None and lost_for <= f.coast_s:
                # 刚丢失：沿手原来的移动方向继续追，速度逐渐衰减
                rt.aim = np.clip(rt.aim + rt.aim_vel * dt, lo, hi)
                rt.aim_vel = rt.aim_vel * max(0.0, 1.0 - dt / f.coast_s)
                goal, rt.status = rt.aim, "短暂丢失，继续追"
            elif lost_for <= f.lost_timeout_s:
                goal, rt.status = (rt.aim if rt.aim is not None else current), "短暂丢失，保持"
            else:
                goal, rt.aim = np.zeros(2), None
                rt.status = "未见手，回初始位" if float(np.max(np.abs(current))) > 0.5 else "等待手出现"

        returning = hand is None and not guided and (yielding or rt.aim is None)
        top = self.cfg.safety.home_speed_deg_s if returning else f.max_speed_deg_s
        rate = np.clip(f.gain * (np.clip(goal, lo, hi) - current), -top, top)
        rt.pan_off, rt.tilt_off = (float(v) for v in np.clip(current + rate * dt, lo, hi))
        desired = dict(rt.home)
        desired["shoulder_pan"] += rt.pan_off
        desired["wrist_flex"] += rt.tilt_off
        return desired

    # ------------------------------------------------------------------ 第 3 步：深度引导
    def _fused_select(self) -> None:
        """用深度相机的主手位置决定由哪只臂去跟。"""
        self._fused_point = None
        cam = self.cameras.get(self.cfg.head_camera)
        hand = cam.primary_hand() if cam else None
        fc = self.cfg.fused
        if hand is None or hand.xyz is None or not (fc.depth_range[0] <= hand.xyz[2] <= fc.depth_range[1]):
            return
        if not self._head_matches_preset():
            return
        best: tuple[float, str, np.ndarray] | None = None
        for name, rt in self.arms.items():
            ext = self.calibration.arms.get(name)
            if ext is None or not rt.enabled or not rt.arm.connected or not rt.arm.torque_enabled:
                continue
            p = ext.T_base_cam[:3, :3] @ hand.xyz + ext.T_base_cam[:3, 3]
            if p[0] < 0.02:  # 手在基座后方，不去够
                continue
            dist = float(np.linalg.norm(p - self.shoulder))
            if name == self._fused_arm:
                dist -= fc.switch_hysteresis  # 当前臂有优先权，避免两臂来回抢
            if best is None or dist < best[0]:
                best = (dist, name, p)
        if best is not None:
            self._fused_arm, self._fused_point = best[1], best[2]

    def _head_matches_preset(self) -> bool:
        preset = self.calibration.head_preset
        if not preset or not self.head_pos:
            return True  # 没有云台或没记录预设时不做限制
        return all(abs(self.head_pos.get(k, 0.0) - preset.get(k, 0.0)) < 3.0 for k in ("pan", "tilt"))

    def _fused(self, name: str, rt: ArmRuntime, dt: float, now: float) -> dict[str, float]:
        fc, s = self.cfg.fused, self.cfg.safety
        if name not in self.calibration.arms:
            desired = self._follow(rt, dt, now)
            rt.status = "未标定·" + rt.status
            return desired

        # 腕部精调：在引导姿态上叠加小角度修正
        trim = (-fc.wrist_trim_limit_deg, fc.wrist_trim_limit_deg)
        err = self._wrist_error(rt)
        if err is not None:
            self._apply_wrist_error(rt, err, dt, trim, trim)
        else:
            step = 10.0 * dt
            rt.pan_off = _approach(rt.pan_off, 0.0, step)
            rt.tilt_off = _approach(rt.tilt_off, 0.0, step)

        if rt.guided is None:
            rt.guided = {j: rt.home[j] for j in ARM_JOINTS}
        point = self._fused_point if self._fused_arm == name else None
        if point is not None:
            rt.lost_since = None
            d = point - self.shoulder
            dist = float(np.linalg.norm(d))
            reach = float(np.clip(dist - fc.standoff, fc.min_reach, fc.max_reach))
            goal = self._clamp_workspace(rt, self.shoulder + d / dist * reach)
            look = point - goal
            pitch_goal = float(
                np.clip(np.rad2deg(np.arctan2(look[2], np.hypot(look[0], look[1]))), *s.pitch_range_deg)
            )
            if rt.cart is None:
                tip, pitch = self.kin.tip_and_pitch(rt.guided)
                rt.cart = CartTarget(tip, pitch)
            move = goal - rt.cart.pos
            max_move = fc.linear_speed * dt
            if np.linalg.norm(move) > max_move:
                move *= max_move / np.linalg.norm(move)
            cand_pitch = _approach(rt.cart.pitch, pitch_goal, self.cfg.keyboard.pitch_speed_deg_s * dt)
            # 位置和俯仰一起走不通时分别尝试，避免目标略超可达范围就整体卡住
            seed = rt.guided
            for pos, pitch in (
                (rt.cart.pos + move, cand_pitch),
                (rt.cart.pos + move, rt.cart.pitch),
                (rt.cart.pos, cand_pitch),
            ):
                res = self._solve(rt, pos, pitch, seed)
                if res is not None:
                    rt.cart = CartTarget(pos, pitch)
                    rt.guided.update(res.joints)
                    break
            rt.status = "深度引导+腕部精调" if err is not None else "深度引导"
        elif err is not None:
            rt.lost_since = None
            rt.status = "腕部精调"
        else:
            rt.lost_since = rt.lost_since or now
            if now - rt.lost_since > fc.lost_timeout_s:
                step = s.home_speed_deg_s * dt
                rt.guided = {j: _approach(rt.guided[j], rt.home[j], step) for j in ARM_JOINTS}
                rt.cart = None
                rt.status = "等待手出现"
            else:
                rt.status = "短暂丢失，保持"

        desired = dict(rt.home)
        desired.update(rt.guided)
        desired["shoulder_pan"] += rt.pan_off
        desired["wrist_flex"] += rt.tilt_off
        return desired

    # ------------------------------------------------------------------ 云台
    def _tick_head(self, keys: set[str], dt: float) -> None:
        head = self.head
        if head is None or not head.connected or self.head_target is None:
            return
        try:
            self.head_pos = head.read()
            if self.estop:
                return
            if self.mode == "keyboard":
                for code, (axis, sign) in HEAD_KEYMAP.items():
                    if code in keys:
                        self.head_target[axis] += sign * self.cfg.head.speed_deg_s * dt
            for axis, (lo, hi) in head.limits().items():
                self.head_target[axis] = float(np.clip(self.head_target[axis], lo, hi))
            head.write(self.head_target)
        except Exception as e:
            head.error = f"通信失败: {e}"
            head.connected = False
            logger.error("云台通信失败: %s", e)

    def head_to_preset(self) -> None:
        with self._lock:
            if self.calibration.head_preset and self.head_target is not None:
                self.head_target = dict(self.calibration.head_preset)

    # ------------------------------------------------------------------ 状态输出
    def _update_overlays(self) -> None:
        for name, rt in self.arms.items():
            cam = self.cameras.get(rt.cfg.camera)
            if cam is None:
                continue
            active = self.mode in ("follow", "fused") and rt.goto is None and not self.estop
            cam.overlay_active = active
            cam.deadzone = self.cfg.follow.deadzone if active else 0.0
        cam = self.cameras.get(self.cfg.head_camera)
        if cam is not None:
            # 腕部跟随本身用不到头部相机的检测；只有要自动标定外参时才保留
            cam.detect_enabled = self.mode != "follow" or self.cfg.fused.auto_calibrate
            guiding = self._fused_arm if self.cfg.fused.reach else self._cue_arm
            cam.overlay_active = self.mode == "fused" and guiding is not None and (
                self._fused_point is not None or bool(self._cues)
            )
            cam.overlay_text = f"-> {guiding}" if cam.overlay_active else ""

    def _is_tracking(self, name: str, rt: ArmRuntime) -> bool:
        """转动对准的模式下，这只臂当前是否正在跟一只手或被深度引导带着转（用于界面高亮）。"""
        if not self._pointing_mode():
            return False
        if self.cfg.follow.single_tracker:
            return self._follow_arm == name
        return rt.aim is not None and rt.lost_since is None

    @staticmethod
    def _hand_info(hand) -> dict | None:
        """主手的画面位置与（深度相机上的）3D 坐标，供界面显示和现场排查。"""
        if hand is None:
            return None
        return {
            "cx": round(hand.cx, 3),
            "cy": round(hand.cy, 3),
            "size": round(hand.size, 3),
            "score": round(hand.score, 2),
            "xyz": None if hand.xyz is None else [round(float(v), 3) for v in hand.xyz],
        }

    def snapshot(self) -> dict:
        with self._lock:
            arms = {}
            for name, rt in self.arms.items():
                tip, pitch = (None, None)
                if rt.pos:
                    t, pitch = self.kin.tip_and_pitch({j: rt.pos[j] for j in ARM_JOINTS})
                    tip = [round(float(v), 3) for v in t]
                ext = self.calibration.arms.get(name)
                arms[name] = {
                    "connected": rt.arm.connected,
                    "enabled": rt.enabled,
                    "lent": rt.lent,
                    "error": rt.arm.error,
                    "torque": rt.arm.torque_enabled,
                    "status": rt.status,
                    "joints": {j: round(v, 1) for j, v in rt.pos.items()},
                    "target": {j: round(v, 1) for j, v in rt.target.items()},
                    "limits": {j: [round(v, 1) for v in lim] for j, lim in rt.limits.items()},
                    "tip": tip,
                    "pitch": None if pitch is None else round(pitch, 1),
                    "moving_to": rt.goto_name if rt.goto is not None else None,
                    "calibrated": ext is not None,
                    "calib_rms_mm": None if ext is None else round(ext.rms * 1000, 1),
                    "calib_samples": self.calib_session.count(name),
                    "fused_selected": (self.mode == "fused" and self.cfg.fused.reach and self._fused_arm == name
                                       and self._fused_point is not None)
                    or self._is_tracking(name, rt),
                    "auto_samples": len(self.auto_samples[name]),
                    "view_quality": round(self._view_quality(rt), 2),
                }
            cams = {
                name: {
                    "primary": self._hand_info(cam.primary_hand()),
                    "error": cam.error,
                    "detect_error": cam.detect_error,
                    "fps": round(cam.fps, 1),
                    "detect_fps": round(cam.detect_fps, 1),
                    "read_ms": round(cam.read_ms, 1),
                    "detect_ms": round(cam.detect_ms, 1),
                    "size": [cam.cfg.width, cam.cfg.height],
                    "hands": cam.hand_count(),
                    "locked": cam.primary_hand() is not None,
                    "depth": cam.cfg.type == "realsense",
                }
                for name, cam in self.cameras.items()
            }
            head = None
            if self.head is not None:
                head = {
                    "connected": self.head.connected,
                    "error": self.head.error,
                    "pos": {k: round(v, 1) for k, v in self.head_pos.items()},
                    "preset": self.calibration.head_preset,
                    "matches_preset": self._head_matches_preset(),
                }
            return {
                "mode": self.mode,
                "estop": self.estop,
                "mock": self.cfg.mock,
                "loop_hz": round(self.loop_hz, 1),
                "keys": sorted(self._keys),
                "message": self.message,
                "arms": arms,
                "head": head,
                "cameras": cams,
                "head_camera": self.cfg.head_camera,
                "guidance_hits": list(self.guidance_hits)[-20:],
                "floor_pitch_deg": None if self._floor_up is None
                else round(float(np.degrees(np.arcsin(np.clip(-self._floor_up[2], -1, 1)))), 1),
            }
