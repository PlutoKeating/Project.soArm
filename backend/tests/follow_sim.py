"""腕部跟随的闭环模拟：同一段手部运动，带检测噪声、观测延迟和舵机滞后，用来对比不同控制器。

直接运行可打印对比表：python backend/tests/follow_sim.py
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from soarm.config import CameraConfig, Config  # noqa: E402
from soarm.control.controller import Controller  # noqa: E402
from soarm.perception.hand import Hand  # noqa: E402
from soarm.robot.base import JOINTS, Arm  # noqa: E402
from soarm.robot.mock import MockHead  # noqa: E402

TICK = 1 / 30  # 控制周期
FRAME = 1 / 30  # 相机帧间隔


class ServoArm(Arm):
    """比 MockArm 更接近真实舵机：每个关节以一阶滞后趋向目标，并有最高转速。"""

    def __init__(self, name: str, start: dict[str, float], tau: float = 0.06, max_speed: float = 200.0):
        super().__init__(name)
        self.pos, self.target = dict(start), dict(start)
        self.tau, self.max_speed = tau, max_speed

    def connect(self) -> None:
        self.connected = self.torque_enabled = True

    def disconnect(self) -> None:
        self.connected = False

    def advance(self, dt: float) -> None:
        for j in JOINTS:
            want = (self.target[j] - self.pos[j]) / self.tau
            self.pos[j] += float(np.clip(want, -self.max_speed, self.max_speed)) * dt

    def read(self) -> dict[str, float]:
        return dict(self.pos)

    def write(self, target: dict[str, float]) -> None:
        self.target.update(target)

    def set_torque(self, enabled: bool) -> None:
        self.torque_enabled = enabled


class DelayedCamera:
    def __init__(self):
        self.cfg = CameraConfig(device="0", width=1280, height=720, hfov_deg=90.0)
        self.hand = None
        self.error = self.detect_error = None
        self.fps = self.detect_fps = 30.0
        self.read_ms = self.detect_ms = 0.0
        self.detect_enabled, self.overlay_active, self.overlay_text, self.deadzone = True, False, "", 0.0

    def primary_hand(self, max_age_s: float = 0.4):
        return self.hand

    def hand_count(self):
        return 0 if self.hand is None else 1


def hand_path(t: float) -> np.ndarray:
    """手所在方向对应的 (水平, 俯仰) 偏转量（度）。分段：静止、慢速往复、静止、匀速扫动、快速往复、静止。"""
    c = -10.0  # 水平方向的中心，留在左臂的可转范围内
    if t < 4:
        return np.array([c, 5.0])
    if t < 10:  # 0.5 Hz，幅度 18°，峰值约 57°/秒
        s = t - 4
        return np.array([c + 18 * np.sin(2 * np.pi * 0.5 * s), 5 + 8 * np.sin(2 * np.pi * 0.5 * s + 1.0)])
    if t < 13:
        return np.array([c, 5 + 8 * np.sin(1.0)])
    if t < 17:  # 匀速来回扫，40°/秒
        s = (t - 13) % 2.0
        x = 40 * s if s < 1.0 else 40 * (2.0 - s)
        return np.array([c - 20 + x, 5 + 8 * np.sin(1.0)])
    if t < 21:  # 1 Hz，幅度 12°，峰值约 75°/秒
        s = t - 17
        return np.array([c - 20 + 12 * np.sin(2 * np.pi * 1.0 * s), 5 + 8 * np.sin(1.0) + 5 * np.sin(2 * np.pi * 1.0 * s)])
    if t < 24:
        return np.array([c - 20, 5 + 8 * np.sin(1.0)])
    if t < 30:
        # 人手“停着”时的样子：缓慢漂移（3°/秒）加一点轻微颤动。硬死区在这种输入下会走走停停
        s = t - 24
        drift = 3.0 * s if s < 3 else 3.0 * (6 - s)
        return np.array([c - 20 + drift + 0.4 * np.sin(2 * np.pi * 2.5 * s), 5 + 8 * np.sin(1.0) + 0.3 * np.sin(2 * np.pi * 1.7 * s)])
    # 中等速度的往复（0.3 Hz，幅度 12°，峰值约 23°/秒）：真人互动里最常见的速度段
    s = t - 30
    return np.array([c - 20 + 12 * np.sin(2 * np.pi * 0.3 * s), 5 + 8 * np.sin(1.0) + 6 * np.sin(2 * np.pi * 0.3 * s + 0.7)])


STILL = [(1.5, 4.0), (11.0, 13.0), (22.0, 24.0)]  # 手静止且已稳定的时间段
MOVING = [(4.5, 10.0), (13.5, 17.0), (17.5, 21.0)]
SLOW = [(24.5, 30.0)]  # 缓慢漂移
MODERATE = [(31.0, 40.0)]  # 中等速度往复
DURATION = 40.0


@dataclass
class Result:
    still_jitter: float  # 手静止时手臂实际角度的抖动（标准差，度）
    roughness: float  # 指令角加速度的均方根（度/秒²）
    lag_rms: float  # 手运动时“手的方向 - 手臂朝向”的均方根（度）
    lag_ms: float  # 手臂相对手的时间滞后（毫秒），由互相关估计
    max_error: float  # 运动中的最大偏差（度）
    lost_frames: int  # 手落到腕部画面之外的帧数
    overshoot: float  # 匀速扫动掉头处手臂冲过头的最大角度（度）
    slow_roughness: float  # 手缓慢漂移时手臂实际角速度的起伏（标准差，度/秒）：走走停停会很大
    slow_error: float  # 手缓慢漂移时的偏差均方根（度）
    moderate_error: float  # 中等速度往复时的偏差均方根（度）


def simulate(controller: str, noise: float = 0.012, latency: float = 0.09, seed: int = 0, tmp: str = "/tmp",
             overrides: dict | None = None) -> Result:
    """noise 是归一化画面坐标（半幅为 1）上的检测噪声标准差；latency 是从成像到控制器拿到结果的延迟。"""
    cfg = Config(mock=True, calibration_dir=tmp)
    cfg.follow.controller = controller
    cfg.follow.aim_scale = 1.0
    for key, value in (overrides or {}).items():
        setattr(cfg.follow, key, value)
    cams = {"wrist_left": DelayedCamera(), "wrist_right": DelayedCamera(), "head": DelayedCamera()}
    home = dict(cfg.home)
    arms = {"left": ServoArm("left", home), "right": ServoArm("right", home)}
    ctl = Controller(cfg, arms, MockHead(), cams)
    ctl.connect_all()
    ctl.set_mode("follow")
    for rt in ctl.arms.values():
        rt.goto = None  # 已经在初始位
    rt = ctl.arms["left"]
    cam = cams["wrist_left"]
    rng = np.random.default_rng(seed)
    half = np.array([cam.cfg.hfov_deg / 2.0, cam.cfg.hfov_deg * cam.cfg.height / cam.cfg.width / 2.0])
    signs = np.array([ctl._pan_dir, ctl._tilt_dir])

    sub = 6  # 每个控制周期里把舵机推进 6 小步
    h = TICK / sub
    t, next_frame = 0.0, 0.0
    frames: list[tuple[float, float, np.ndarray]] = []  # (成像时刻, 可用时刻, 归一化画面坐标)
    log = []
    cmd_prev = cmd_prev_vel = None
    rough = []
    lost = 0
    while t < DURATION:
        for _ in range(sub):
            if t >= next_frame:
                offset = np.array([arms["left"].pos["shoulder_pan"] - home["shoulder_pan"],
                                   arms["left"].pos["wrist_flex"] - home["wrist_flex"]])
                e = -(hand_path(t) - offset) * signs / half + rng.normal(0, noise, 2)
                frames.append((t, t + latency, e))
                next_frame += FRAME
            for arm in arms.values():
                arm.advance(h)
            t += h
        ready = [fr for fr in frames if fr[1] <= t]
        if ready:
            shot, _, e = ready[-1]
            if np.all(np.abs(e) <= 1.0):
                cam.hand = Hand(0.5 + e[0] / 2, 0.5 + e[1] / 2, 0.4, 0.9, np.zeros((21, 2), np.float32), None, 100.0 + shot)
            else:
                cam.hand = None
                lost += 1
        ctl._tick(TICK, 100.0 + t)
        offset = np.array([arms["left"].pos["shoulder_pan"] - home["shoulder_pan"],
                           arms["left"].pos["wrist_flex"] - home["wrist_flex"]])
        cmd = np.array([rt.pan_off, rt.tilt_off])
        if cmd_prev is not None:
            v = (cmd - cmd_prev) / TICK
            if cmd_prev_vel is not None:
                rough.append(float(np.sum(((v - cmd_prev_vel) / TICK) ** 2)))
            cmd_prev_vel = v
        cmd_prev = cmd
        log.append((t, *hand_path(t), *offset))
    log = np.array(log)
    ts, hand, arm = log[:, 0], log[:, 1:3], log[:, 3:5]

    def mask(spans):
        m = np.zeros(len(ts), bool)
        for a, b in spans:
            m |= (ts >= a) & (ts < b)
        return m

    still, moving = mask(STILL), mask(MOVING)
    jitter = float(np.sqrt(np.mean([arm[(ts >= a) & (ts < b)].std(axis=0) ** 2 for a, b in STILL])))
    err = hand[moving] - arm[moving]
    # 时间滞后：把手的水平轨迹往后挪若干毫秒，看挪多少和手臂的轨迹最吻合
    best_lag, best_cost = 0.0, np.inf
    for lag in np.arange(0.0, 0.4, 0.005):
        shifted = np.array([hand_path(x - lag)[0] for x in ts[moving]])
        cost = float(np.mean((shifted - arm[moving][:, 0]) ** 2))
        if cost < best_cost:
            best_lag, best_cost = lag, cost
    sweep = (ts >= 13.5) & (ts < 17.0)
    over = float(max(arm[sweep][:, 0].max() - hand[sweep][:, 0].max(), hand[sweep][:, 0].min() - arm[sweep][:, 0].min(), 0.0))
    slow = mask(SLOW)
    arm_speed = np.diff(arm[slow][:, 0]) / TICK
    hand_speed = np.diff(hand[slow][:, 0]) / TICK
    slow_rough = float(np.std(arm_speed - hand_speed))
    slow_err = float(np.sqrt(np.mean((hand[slow] - arm[slow]) ** 2)))
    mod = mask(MODERATE)
    mod_err = float(np.sqrt(np.mean((hand[mod] - arm[mod]) ** 2)))
    return Result(jitter, float(np.sqrt(np.mean(rough))), float(np.sqrt(np.mean(err**2))), best_lag * 1000,
                  float(np.abs(err).max()), lost, over, slow_rough, slow_err, mod_err)


def main() -> None:
    import tempfile

    tmp = tempfile.mkdtemp()
    fields = [("still_jitter", "静止抖动 °"), ("slow_roughness", "慢漂时速度起伏 °/s"), ("slow_error", "慢漂时偏差 °"),
              ("moderate_error", "中速往复偏差 °"),
              ("roughness", "指令粗糙度 °/s²"), ("lag_rms", "运动偏差 °"), ("lag_ms", "时间滞后 ms"),
              ("max_error", "最大偏差 °"), ("overshoot", "掉头冲过头 °"), ("lost_frames", "丢失帧")]
    results = {}
    latency = float(sys.argv[1]) if len(sys.argv) > 1 else 0.09
    print(f"观测延迟 {latency * 1000:.0f} ms")
    for name in ("legacy", "smooth"):
        runs = [simulate(name, seed=s, tmp=tmp, latency=latency) for s in range(4)]
        results[name] = {f: float(np.mean([getattr(r, f) for r in runs])) for f, _ in fields}
    print(f"{'指标':<22}{'旧 legacy':>12}{'新 smooth':>12}{'变化':>10}")
    for f, label in fields:
        old, new = results["legacy"][f], results["smooth"][f]
        change = f"{(new - old) / old * 100:+.0f}%" if old > 1e-9 else "-"
        print(f"{label:<22}{old:>12.2f}{new:>12.2f}{change:>10}")


if __name__ == "__main__":
    main()
