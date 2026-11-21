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
from dataclasses import dataclass, field

import numpy as np

from ..calib import Calibration, CalibSession
from ..config import ArmConfig, Config
from ..kinematics import ARM_JOINTS, IK_JOINTS, ArmKinematics
from ..perception.cameras import CameraWorker
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
    status: str = ""


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
        self.arms = {name: ArmRuntime(arm, cfg.arms[name]) for name, arm in arms.items()}
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

    def start(self) -> None:
        self.connect_all()
        self._thread.start()

    def connect_all(self) -> None:
        """连接尚未连上的设备。失败只记录错误，不影响其它设备。"""
        with self._lock:
            for name, rt in self.arms.items():
                if rt.arm.connected:
                    continue
                try:
                    rt.arm.connect()
                    rt.pos = rt.arm.read()
                    rt.target = dict(rt.pos)  # 以当前位置为目标，上电不跳变
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
                    self._start_goto(rt, self.cfg.rest, "rest", release=True)
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
            for rt in self.arms.values():
                self._reset_motion(rt)
                if mode in ("follow", "fused") or (mode == "keyboard" and not self._in_workspace(rt)):
                    self._start_goto(rt, self.home, "home")
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
        target = {"home": self.home, "rest": self.cfg.rest}.get(pose)
        if target is None:
            raise ValueError(f"未知姿态 {pose}")
        with self._lock:
            if self.estop:
                raise RuntimeError("急停中，请先解除急停")
            self.mode = "idle"
            for rt in self.arms.values():
                self._reset_motion(rt)
                self._start_goto(rt, target, pose, release=release)

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

    def _start_goto(self, rt: ArmRuntime, pose: dict[str, float], name: str, release: bool = False) -> None:
        if not rt.arm.connected or not rt.arm.torque_enabled:
            return
        rt.goto = dict(pose)
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
        if self.mode == "fused" and not self.estop:
            self._fused_select()
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
        if worst > safety.stall_error_deg:
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
        if rt.goto is not None:
            desired, speed = dict(rt.goto), safety.home_speed_deg_s
            rt.status = "回初始位" if rt.goto_name == "home" else "收到休息位"
        elif self.mode == "keyboard":
            desired = self._keyboard(name, rt, keys, dt)
        elif self.mode == "follow":
            desired = self._follow(rt, dt, now)
        elif self.mode == "fused":
            desired = self._fused(name, rt, dt, now)
        else:
            desired = dict(rt.target)
            rt.status = rt.status or "保持"

        max_step = speed * dt
        new = {}
        for j in JOINTS:
            step = max_step * (2.5 if j == "gripper" else 1.0)
            new[j] = _approach(rt.target[j], desired.get(j, rt.target[j]), step)
        arm_part = self.kin.clamp({j: new[j] for j in ARM_JOINTS}, safety.joint_limit_margin_deg)
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
        res = self.kin.ik(pos, pitch, {j: seed[j] for j in ARM_JOINTS})
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

    def _follow(self, rt: ArmRuntime, dt: float, now: float) -> dict[str, float]:
        f = self.cfg.follow
        err = self._wrist_error(rt)
        if err is not None:
            rt.lost_since = None
            self._apply_wrist_error(rt, err, dt, f.pan_range_deg, f.tilt_range_deg)
            rt.status = "跟随中" if (err[0] or err[1]) else "已对准"
        else:
            rt.lost_since = rt.lost_since or now
            if now - rt.lost_since > f.lost_timeout_s:
                step = self.cfg.safety.home_speed_deg_s * dt
                rt.pan_off = _approach(rt.pan_off, 0.0, step)
                rt.tilt_off = _approach(rt.tilt_off, 0.0, step)
                rt.status = "未见手，回初始位" if (rt.pan_off or rt.tilt_off) else "等待手出现"
            else:
                rt.status = "短暂丢失，保持"
        desired = dict(self.home)
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
            if ext is None or not rt.arm.connected or not rt.arm.torque_enabled:
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
            rt.guided = {j: self.home[j] for j in ARM_JOINTS}
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
            seed = {**rt.guided, "wrist_roll": 0.0}
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
                rt.guided = {j: _approach(rt.guided[j], self.home[j], step) for j in ARM_JOINTS}
                rt.cart = None
                rt.status = "等待手出现"
            else:
                rt.status = "短暂丢失，保持"

        desired = dict(self.home)
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
            cam.overlay_active = self.mode == "fused" and self._fused_point is not None
            cam.overlay_text = f"-> {self._fused_arm}" if cam.overlay_active else ""

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
                    "error": rt.arm.error,
                    "torque": rt.arm.torque_enabled,
                    "status": rt.status,
                    "joints": {j: round(v, 1) for j, v in rt.pos.items()},
                    "target": {j: round(v, 1) for j, v in rt.target.items()},
                    "tip": tip,
                    "pitch": None if pitch is None else round(pitch, 1),
                    "moving_to": rt.goto_name if rt.goto is not None else None,
                    "calibrated": ext is not None,
                    "calib_rms_mm": None if ext is None else round(ext.rms * 1000, 1),
                    "calib_samples": self.calib_session.count(name),
                    "fused_selected": self.mode == "fused" and self._fused_arm == name
                    and self._fused_point is not None,
                }
            cams = {
                name: {
                    "error": cam.error,
                    "detect_error": cam.detect_error,
                    "fps": round(cam.fps, 1),
                    "detect_fps": round(cam.detect_fps, 1),
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
            }
