import numpy as np
import pytest

from soarm.calib import solve_rigid
from soarm.config import Config
from soarm.kinematics import ARM_JOINTS, ArmKinematics


@pytest.fixture(scope="module")
def kin():
    return ArmKinematics()


def test_chain_has_five_arm_joints(kin):
    assert [j.name for j in kin.chain if not j.fixed] == ARM_JOINTS


def test_home_and_rest_poses_are_sensible(kin):
    cfg = Config()
    tip, pitch = kin.tip_and_pitch(cfg.home)
    s = cfg.safety
    # 初始位：夹爪在基座前方、工作空间内、大致水平朝前
    assert s.x_range[0] < tip[0] < s.x_range[1]
    assert abs(tip[1]) < 0.02
    assert s.z_range[0] < tip[2] < s.z_range[1]
    assert abs(pitch) < 25
    assert kin.approach_dir(cfg.home)[0] > 0.8
    # 休息位：收拢在基座附近且比初始位低
    rest_tip, _ = kin.tip_and_pitch(cfg.rest)
    assert np.linalg.norm(rest_tip[:2]) < np.linalg.norm(tip[:2])
    for pose in (cfg.home, cfg.rest):
        for j in ARM_JOINTS:
            lo, hi = kin.limits[j]
            assert lo < pose[j] < hi


def test_pan_is_symmetric_and_lift_lowers_tip(kin):
    base = {j: 0.0 for j in ARM_JOINTS}
    left, _ = kin.tip_and_pitch({**base, "shoulder_pan": 20.0})
    right, _ = kin.tip_and_pitch({**base, "shoulder_pan": -20.0})
    assert left[1] == pytest.approx(-right[1], abs=1e-3) and abs(left[1]) > 0.05
    zero, _ = kin.tip_and_pitch(base)
    leaned, _ = kin.tip_and_pitch({**base, "shoulder_lift": 20.0})
    assert leaned[2] < zero[2]


def test_ik_roundtrip(kin):
    rng = np.random.default_rng(0)
    home = Config().home
    solved = 0
    for _ in range(200):
        q = {
            "shoulder_pan": rng.uniform(-60, 60),
            "shoulder_lift": rng.uniform(-90, 10),
            "elbow_flex": rng.uniform(0, 85),
            "wrist_flex": rng.uniform(-50, 50),
            "wrist_roll": home["wrist_roll"],  # 腕部旋转不参与逆解，目标与种子取同一个值
        }
        tip, pitch = kin.tip_and_pitch(q)
        res = kin.ik(tip, pitch, {j: home[j] for j in ARM_JOINTS}, max_iters=80)
        if res.pos_error < 0.003 and res.pitch_error < 2.0:
            solved += 1
    assert solved >= 190  # 从初始位出发，绝大多数可达位姿都应能解出


def test_ik_small_step_is_accurate(kin):
    home = {j: Config().home[j] for j in ARM_JOINTS}
    tip, pitch = kin.tip_and_pitch(home)
    for delta in ([0.005, 0, 0], [0, 0.005, 0], [0, 0, 0.005]):
        res = kin.ik(tip + np.array(delta), pitch, home)
        assert res.pos_error < 0.001
        assert res.pitch_error < 1.0
        # 小位移只应引起小的关节变化
        assert max(abs(res.joints[j] - home[j]) for j in res.joints) < 5.0


def test_ik_reports_unreachable(kin):
    home = {j: Config().home[j] for j in ARM_JOINTS}
    res = kin.ik(np.array([0.8, 0.0, 0.2]), 0.0, home)
    assert res.pos_error > 0.1


def test_solve_rigid_recovers_transform():
    rng = np.random.default_rng(1)
    angle = 0.7
    R = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    R = R @ np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    t = np.array([0.1, -0.2, 0.5])
    cam = rng.uniform(-0.3, 0.3, size=(8, 3))
    base = cam @ R.T + t + rng.normal(0, 0.002, size=(8, 3))
    T, rms = solve_rigid(cam, base)
    assert np.allclose(T[:3, :3], R, atol=0.03)
    assert np.allclose(T[:3, 3], t, atol=0.01)
    assert rms < 0.006


def test_solve_rigid_rejects_degenerate():
    cam = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]], float)
    with pytest.raises(ValueError):
        solve_rigid(cam, cam)
    with pytest.raises(ValueError):
        solve_rigid(cam[:3], cam[:3])
