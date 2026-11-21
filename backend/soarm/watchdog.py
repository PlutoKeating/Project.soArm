"""无人值守看门狗：独立进程，只读轮询运行中控制台的状态，发现明显异常就让机械臂安全停下。

停下的动作：收到休息位并释放力矩 → 急停。控制台自己已经失去响应时，改为停止服务
（服务退出时同样会先收手再释放力矩）。

判定条件都要“持续一小段时间”才触发，避免把正常跟随时的瞬时滞后、单次漏检当成异常。
"""

from __future__ import annotations

import json
import subprocess
import time
import urllib.request
from collections import deque
from dataclasses import dataclass, field

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


@dataclass
class Limits:
    tracking_error_deg: float = 30.0  # 实测与目标的偏差
    tracking_error_hold_s: float = 1.5
    blocked_events: int = 3  # 这么多次“受到阻挡”
    blocked_window_s: float = 60.0
    out_of_limit_deg: float = 6.0  # 关节超出本臂限位这么多
    out_of_limit_hold_s: float = 1.0
    min_loop_hz: float = 15.0
    slow_loop_hold_s: float = 3.0
    unreachable_s: float = 6.0  # 状态接口连续这么久读不到
    min_tip_height_m: float = 0.0  # 夹爪参考点低于基座平面（展开状态下）
    low_tip_hold_s: float = 1.0
    disconnected_hold_s: float = 3.0


@dataclass
class Detector:
    """逐帧喂入状态快照，返回触发原因（字符串）或 None。不做任何 I/O，便于测试。"""

    limits: Limits = field(default_factory=Limits)
    _since: dict = field(default_factory=dict)  # 条件名 -> 开始成立的时刻
    _blocked: deque = field(default_factory=deque)
    _was_blocked: dict = field(default_factory=dict)
    _last_ok: float | None = None
    _restarts: int | None = None

    def _held(self, key: str, active: bool, now: float, hold: float) -> bool:
        if not active:
            self._since.pop(key, None)
            return False
        start = self._since.setdefault(key, now)
        return now - start >= hold

    def feed(self, state: dict | None, now: float, restarts: int | None = None) -> str | None:
        lim = self.limits
        if restarts is not None:
            if self._restarts is not None and restarts > self._restarts:
                return f"控制台服务异常重启过（重启次数 {self._restarts} -> {restarts}）"
            self._restarts = restarts
        if state is None:
            self._last_ok = self._last_ok if self._last_ok is not None else now
            if now - self._last_ok >= lim.unreachable_s:
                return f"控制台状态接口已连续 {now - self._last_ok:.0f} 秒没有响应"
            return None
        self._last_ok = now
        if state.get("estop"):
            return None  # 已经急停，不需要再做什么
        if self._held("loop", state.get("loop_hz", 0) < lim.min_loop_hz, now, lim.slow_loop_hold_s):
            return f"控制环频率过低（{state.get('loop_hz')} Hz）"

        for name, arm in state.get("arms", {}).items():
            if arm.get("lent") or not arm.get("enabled", True):
                continue
            if self._held(f"{name}:disconnected", not arm.get("connected"), now, lim.disconnected_hold_s):
                return f"{name} 臂与控制台断开：{arm.get('error') or '未知原因'}"
            if not arm.get("connected") or not arm.get("torque"):
                continue
            joints, target, limits = arm.get("joints") or {}, arm.get("target") or {}, arm.get("limits") or {}
            if not joints:
                continue
            # 回位途中（展开或收手）速度慢、偏差小，不单独处理；这里只看偏差本身
            worst = max((abs(joints[j] - target.get(j, joints[j])) for j in ARM_JOINTS if j in joints), default=0.0)
            if self._held(f"{name}:tracking", worst > lim.tracking_error_deg, now, lim.tracking_error_hold_s):
                return f"{name} 臂实际位置与指令偏差 {worst:.0f}°，持续超过 {lim.tracking_error_hold_s} 秒"
            over = 0.0
            for j, (lo, hi) in limits.items():
                if j in joints:
                    over = max(over, lo - joints[j], joints[j] - hi)
            # 刚上力矩时腕部可能还停在限位外（收拢停放的姿态），所以只在展开工作时检查
            working = arm.get("moving_to") is None and "力矩" not in (arm.get("status") or "")
            if self._held(f"{name}:limit", working and over > lim.out_of_limit_deg, now, lim.out_of_limit_hold_s):
                return f"{name} 臂有关节超出限位 {over:.0f}°"
            tip = arm.get("tip")
            if self._held(f"{name}:low", working and tip is not None and tip[2] < lim.min_tip_height_m, now,
                          lim.low_tip_hold_s):
                return f"{name} 臂夹爪低于基座平面（高度 {tip[2] * 100:.0f} 厘米）"
            blocked = "阻挡" in (arm.get("status") or "")
            if blocked and not self._was_blocked.get(name):
                self._blocked.append(now)
            self._was_blocked[name] = blocked
        while self._blocked and now - self._blocked[0] > lim.blocked_window_s:
            self._blocked.popleft()
        if len(self._blocked) >= lim.blocked_events:
            return f"{lim.blocked_window_s:.0f} 秒内出现 {len(self._blocked)} 次“受到阻挡”"
        return None


def _get(url: str, timeout: float = 2.0) -> dict | None:
    try:
        return json.load(urllib.request.urlopen(url, timeout=timeout))
    except Exception:
        return None


def _post(url: str, body: dict, timeout: float = 5.0) -> bool:
    try:
        req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"content-type": "application/json"})
        urllib.request.urlopen(req, timeout=timeout)
        return True
    except Exception:
        return False


def _service_restarts(unit: str) -> int | None:
    try:
        out = subprocess.run(["systemctl", "--user", "show", unit, "-p", "NRestarts", "--value"],
                             capture_output=True, text=True, timeout=3).stdout.strip()
        return int(out)
    except (OSError, ValueError, subprocess.SubprocessError):
        return None


def safe_stop(url: str, unit: str | None, log=print) -> str:
    """让机械臂安全停下：收到休息位并释放力矩，然后急停。返回一句结果说明。"""
    state = _get(f"{url}/api/state")
    if state is not None:
        if state.get("estop"):
            _post(f"{url}/api/estop", {"active": False})  # 急停中收不了手，先解除再收
        # 借出去的臂先要回来，否则收不到它
        if any(a.get("lent") for a in state.get("arms", {}).values()):
            _post(f"{url}/api/restore", {}, timeout=15)
        if _post(f"{url}/api/go", {"pose": "rest", "release": True}):
            log("已下发：收到休息位并释放力矩")
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline:
                s = _get(f"{url}/api/state")
                if s is None:
                    break
                live = [a for a in s["arms"].values() if a.get("connected")]
                if all(not a.get("torque") for a in live):
                    break
                time.sleep(0.3)
            s = _get(f"{url}/api/state")
            released = s is not None and all(not a.get("torque") for a in s["arms"].values() if a.get("connected"))
            _post(f"{url}/api/estop", {"active": True})
            log("已急停")
            if released:
                return "已收到休息位、释放力矩并急停"
            return "已急停，但没有确认到力矩全部释放，请到现场检查"
    # 控制台没有响应：停止服务。服务退出时会先收手再释放力矩
    if unit:
        log("控制台没有响应，改为停止服务")
        try:
            subprocess.run(["systemctl", "--user", "stop", unit], timeout=45)
            return "控制台无响应，已停止服务（退出流程会收手并释放力矩），请到现场确认"
        except (OSError, subprocess.SubprocessError) as e:
            return f"控制台无响应，停止服务也失败了：{e}。请立即到现场处理"
    return "控制台没有响应，也没有可停止的服务。请立即到现场处理"


def run(url: str, unit: str | None, interval: float = 0.25, dry_run: bool = False, max_seconds: float = 0.0) -> int:
    detector = Detector()
    start = time.monotonic()
    last_report = 0.0
    print(f"看门狗已启动：监视 {url}" + ("（演练模式，只报告不动作）" if dry_run else ""), flush=True)
    while True:
        now = time.monotonic()
        state = _get(f"{url}/api/state")
        restarts = _service_restarts(unit) if unit else None
        reason = detector.feed(state, now, restarts)
        if reason:
            stamp = time.strftime("%H:%M:%S")
            print(f"[{stamp}] 检测到异常：{reason}", flush=True)
            if dry_run:
                return 3
            print(f"[{time.strftime('%H:%M:%S')}] {safe_stop(url, unit)}", flush=True)
            return 2
        if now - last_report >= 60 and state is not None:
            last_report = now
            arms = "  ".join(f"{n}: {a.get('status') or '-'}" for n, a in state["arms"].items())
            print(f"[{time.strftime('%H:%M:%S')}] 正常  模式 {state['mode']}  {state['loop_hz']} Hz  {arms}", flush=True)
        if max_seconds and now - start >= max_seconds:
            print("到达设定的监视时长，未发现异常。", flush=True)
            return 0
        time.sleep(interval)
