"""看门狗的异常判定与安全停下动作。判定用合成快照测；动作对着一个模拟控制台走一遍。"""

import copy
import threading
import time

import pytest
import uvicorn

from soarm.app import create_app
from soarm.config import Config
from soarm.watchdog import Detector, safe_stop

HEALTHY = {
    "mode": "fused", "estop": False, "loop_hz": 29.9,
    "arms": {
        name: {
            "connected": True, "enabled": True, "lent": False, "torque": True, "status": "等待手出现", "error": None,
            "moving_to": None, "tip": [0.26, 0.0, 0.2],
            "joints": {"shoulder_pan": 0.0, "shoulder_lift": -75.0, "elbow_flex": 64.0, "wrist_flex": 11.0, "wrist_roll": -90.0},
            "target": {"shoulder_pan": 0.0, "shoulder_lift": -75.0, "elbow_flex": 60.0, "wrist_flex": 10.0, "wrist_roll": -90.0},
            "limits": {"shoulder_pan": [-46.6, 27.5], "shoulder_lift": [-95.0, 88.2], "elbow_flex": [-91.8, 91.8],
                       "wrist_flex": [-44.1, 63.0], "wrist_roll": [-152.2, 157.8]},
        }
        for name in ("left", "right")
    },
}


def feed_for(det, state, seconds, start=0.0, restarts=0):
    t, reason = start, None
    while t <= start + seconds and reason is None:
        reason = det.feed(state, t, restarts)
        t += 0.25
    return reason


def test_healthy_state_never_triggers():
    assert feed_for(Detector(), HEALTHY, 120) is None


def test_normal_fast_following_with_brief_lag_does_not_trigger():
    det = Detector()
    lagging = copy.deepcopy(HEALTHY)
    lagging["arms"]["left"]["joints"]["shoulder_pan"] = -25.0  # 瞬时滞后 25°，在阈值内
    lagging["arms"]["left"]["status"] = "跟随中"
    assert feed_for(det, lagging, 30) is None
    spike = copy.deepcopy(HEALTHY)
    spike["arms"]["left"]["joints"]["shoulder_pan"] = -40.0  # 超过阈值但只有一瞬间
    assert det.feed(spike, 31.0, 0) is None and det.feed(spike, 31.5, 0) is None
    assert feed_for(det, HEALTHY, 10, start=32.0) is None


def test_sustained_tracking_error_triggers():
    stuck = copy.deepcopy(HEALTHY)
    stuck["arms"]["right"]["joints"]["shoulder_lift"] = -30.0  # 目标 -75，实际卡在 -30
    reason = feed_for(Detector(), stuck, 5)
    assert reason and "right" in reason and "偏差" in reason


def test_joint_outside_limits_triggers_but_not_while_parking():
    out = copy.deepcopy(HEALTHY)
    out["arms"]["left"]["joints"]["shoulder_pan"] = 40.0  # 上限 27.5
    out["arms"]["left"]["target"]["shoulder_pan"] = 40.0
    assert "限位" in feed_for(Detector(), out, 5)
    parking = copy.deepcopy(out)
    parking["arms"]["left"]["moving_to"] = "rest"
    assert feed_for(Detector(), parking, 5) is None


def test_repeated_blocking_triggers_but_a_single_event_does_not():
    det = Detector()
    blocked = copy.deepcopy(HEALTHY)
    blocked["arms"]["left"]["status"] = "受到阻挡，已让位"
    t = 0.0
    assert det.feed(blocked, t, 0) is None
    for _ in range(2):  # 每次阻挡之间恢复正常
        t += 5
        assert det.feed(HEALTHY, t, 0) is None
        t += 5
        reason = det.feed(blocked, t, 0)
    assert reason and "受到阻挡" in reason
    det2 = Detector()
    det2.feed(blocked, 0.0, 0)
    assert feed_for(det2, HEALTHY, 100, start=1.0) is None


def test_disconnected_arm_low_gripper_and_slow_loop_trigger():
    gone = copy.deepcopy(HEALTHY)
    gone["arms"]["left"].update(connected=False, error="通信失败")
    assert "断开" in feed_for(Detector(), gone, 5)
    low = copy.deepcopy(HEALTHY)
    low["arms"]["right"]["tip"] = [0.3, 0.0, -0.03]
    assert "低于基座" in feed_for(Detector(), low, 5)
    slow = copy.deepcopy(HEALTHY)
    slow["loop_hz"] = 6.0
    assert "控制环" in feed_for(Detector(), slow, 5)


def test_lent_or_disabled_arm_is_ignored():
    lent = copy.deepcopy(HEALTHY)
    lent["arms"]["right"].update(connected=False, lent=True, error="已借给开发调试")
    lent["arms"]["left"].update(enabled=False, connected=False)
    assert feed_for(Detector(), lent, 20) is None


def test_unreachable_console_and_service_restart_trigger():
    det = Detector()
    assert feed_for(det, HEALTHY, 2) is None
    reason = None
    t = 3.0
    while reason is None and t < 20:
        reason = det.feed(None, t, 0)
        t += 0.25
    assert reason and "没有响应" in reason and 8.0 <= t <= 8.5
    det = Detector()
    assert det.feed(HEALTHY, 0.0, 1) is None
    assert "重启" in det.feed(HEALTHY, 1.0, 2)


def test_already_estopped_is_left_alone():
    stopped = copy.deepcopy(HEALTHY)
    stopped["estop"] = True
    stopped["arms"]["left"]["joints"]["shoulder_pan"] = 60.0
    assert feed_for(Detector(), stopped, 30) is None


def test_safe_stop_parks_releases_torque_and_estops(tmp_path):
    cfg = Config(mock=True, cameras={}, calibration_dir=str(tmp_path), start_mode="follow")
    app = create_app(cfg)
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=18766, log_level="error"))
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 10
        while not server.started and time.monotonic() < deadline:
            time.sleep(0.05)
        ctl = app.state.controller
        time.sleep(4.0)  # 模拟臂展开到初始位
        assert ctl.mode == "follow" and all(rt.arm.torque_enabled for rt in ctl.arms.values())
        result = safe_stop("http://127.0.0.1:18766", None, log=lambda *_: None)
        assert result == "已收到休息位、释放力矩并急停"
        assert ctl.estop and ctl.mode == "idle"
        assert all(not rt.arm.torque_enabled for rt in ctl.arms.values())
        for rt in ctl.arms.values():  # 确实回到了收拢的休息位
            assert rt.pos["shoulder_lift"] == pytest.approx(rt.rest["shoulder_lift"], abs=2.0)
    finally:
        server.should_exit = True
        thread.join(timeout=5)


def test_safe_stop_reports_when_console_is_gone():
    result = safe_stop("http://127.0.0.1:18799", None, log=lambda *_: None)
    assert "没有响应" in result
