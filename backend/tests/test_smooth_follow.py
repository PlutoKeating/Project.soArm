"""自适应滤波、二阶平滑跟踪，以及新旧跟随控制器在同一段模拟手部运动下的对比。"""

import numpy as np
import pytest

from follow_sim import simulate
from soarm.control.filters import OneEuroFilter, track_second_order


def test_one_euro_filter_is_heavy_when_still_and_light_when_fast():
    rng = np.random.default_rng(0)
    f = OneEuroFilter(min_cutoff=1.0, beta=0.08, d_cutoff=3.0)
    out = [f.update(np.array([10.0 + rng.normal(0, 0.5)]), i / 30)[0] for i in range(150)]
    assert np.std(out[60:]) < 0.5 * 0.5  # 静止时噪声至少压掉一半
    # 快速匀速运动（60°/秒）时滞后很小
    f = OneEuroFilter(min_cutoff=1.0, beta=0.08, d_cutoff=3.0)
    lag = [60.0 * i / 30 - f.update(np.array([60.0 * i / 30]), i / 30)[0] for i in range(90)]
    assert lag[-1] < 2.0  # 固定 1 Hz 低通在这个速度下会滞后约 9.5°
    assert f.velocity[0] == pytest.approx(60.0, rel=0.05)


def test_second_order_tracking_is_smooth_does_not_overshoot_and_has_no_ramp_lag():
    pos, vel = np.zeros(1), np.zeros(1)
    trace, speeds = [], []
    for _ in range(60):  # 阶跃到 20°
        pos, vel = track_second_order(pos, vel, np.array([20.0]), np.zeros(1), 1 / 30, 0.1, 150.0, 2000.0)
        trace.append(pos[0])
        speeds.append(vel[0])
    assert max(trace) <= 20.0 + 0.05  # 不超调
    assert trace[-1] == pytest.approx(20.0, abs=0.05)
    assert max(abs(np.diff(speeds))) < 2000.0 / 30 + 1e-6  # 速度变化受加速度上限约束，没有速度阶跃
    assert max(speeds) <= 150.0 + 1e-6
    # 目标匀速运动并给出速度前馈：稳态没有滞后
    pos, vel, target = np.zeros(1), np.zeros(1), 0.0
    for _ in range(90):
        pos, vel = track_second_order(pos, vel, np.array([target]), np.array([40.0]), 1 / 30, 0.1, 150.0, 2000.0)
        target += 40.0 / 30
    assert abs(pos[0] - target) < 0.3


@pytest.fixture(scope="module", params=[0.09, 0.15])
def comparison(request, tmp_path_factory):
    """在两种观测延迟下各跑一遍：真机的延迟比最初假设的 90 ms 长，结论要在更长的延迟下也成立。"""
    tmp = str(tmp_path_factory.mktemp("sim"))
    mean = lambda rs, f: float(np.mean([getattr(r, f) for r in rs]))  # noqa: E731
    runs = {
        name: [simulate(name, seed=s, tmp=tmp, latency=request.param) for s in range(3)]
        for name in ("legacy", "smooth")
    }
    return {name: {f: mean(rs, f) for f in vars(rs[0])} for name, rs in runs.items()}


def test_smooth_controller_is_steadier_than_legacy(comparison):
    old, new = comparison["legacy"], comparison["smooth"]
    assert new["still_jitter"] < 0.75 * old["still_jitter"]  # 手不动时手臂更稳
    assert new["roughness"] <= 1.05 * old["roughness"]  # 指令不比旧的更糙


def test_smooth_controller_follows_more_tightly_than_legacy(comparison):
    old, new = comparison["legacy"], comparison["smooth"]
    assert new["moderate_error"] < 0.8 * old["moderate_error"]  # 中等速度：真人互动最常见的速度段
    assert new["lag_rms"] < 0.95 * old["lag_rms"]
    assert new["slow_error"] < 0.8 * old["slow_error"]
    assert new["max_error"] < old["max_error"]
    assert new["lag_ms"] <= old["lag_ms"]
    assert new["lost_frames"] == 0 and new["overshoot"] < 6.0


def test_follow_controller_can_be_switched_at_runtime_and_metrics_are_kept_per_controller(tmp_path):
    from test_controller import Rig, _show

    rig = Rig(tmp_path)
    assert rig.ctl.snapshot()["follow_controller"] == "smooth"  # 默认是新控制
    rig.ctl.set_follow_controller("legacy")
    rig.ctl.set_follow_controller("smooth")
    rig.ctl.set_mode("follow")
    rig.run(6.0)
    assert rig.ctl.snapshot()["follow_controller"] == "smooth"
    _show(rig.cams["wrist_left"], 0.7, 0.5)
    rig.run(2.0)
    rig.ctl.set_follow_controller("legacy")
    rig.run(2.0)
    stats = rig.ctl.snapshot()["follow_stats"]
    assert stats["left:smooth"]["tracking_s"] > 1.5 and stats["left:legacy"]["tracking_s"] > 1.5
    assert stats["left:smooth"]["pixel_error_deg"] is not None
    rig.cams["wrist_left"].hand = None
    rig.run(1.0)
    assert rig.ctl.snapshot()["follow_stats"]["left:legacy"]["lost_events"] == 1
    with pytest.raises(ValueError):
        rig.ctl.set_follow_controller("turbo")


def test_each_arm_can_run_its_own_controller_side_by_side(tmp_path):
    from test_controller import Rig, _show

    rig = Rig(tmp_path)
    rig.ctl.set_follow_controller("legacy", arm="left")
    rig.ctl.set_mode("follow")
    rig.run(6.0)
    snap = rig.ctl.snapshot()
    assert snap["arms"]["left"]["follow_controller"] == "legacy"
    assert snap["arms"]["right"]["follow_controller"] == "smooth"
    _show(rig.cams["wrist_left"], 0.7, 0.5)
    _show(rig.cams["wrist_right"], 0.7, 0.5)
    rig.run(2.0)
    stats = rig.ctl.snapshot()["follow_stats"]
    assert set(stats) == {"left:legacy", "right:smooth"}  # 度量按“臂 × 控制器”分开累计
    assert abs(rig.ctl.arms["left"].pan_off) > 3 and abs(rig.ctl.arms["right"].pan_off) > 3
    # 改全局默认会清掉各臂的单独设置
    rig.ctl.set_follow_controller("legacy")
    assert rig.ctl.snapshot()["arms"]["right"]["follow_controller"] == "legacy"
