"""用模拟臂和假相机逐拍驱动控制器，验证三种模式和安全逻辑。不需要任何硬件。"""

import numpy as np
import pytest

from soarm.calib import ArmExtrinsic
from soarm.config import CameraConfig, Config
from soarm.control.controller import Controller
from soarm.kinematics import ARM_JOINTS
from soarm.perception.hand import Hand
from soarm.robot.mock import MockArm, MockHead

DT = 1 / 30


class FakeCamera:
    def __init__(self, type_: str = "opencv"):
        self.cfg = CameraConfig(type=type_, device="0")
        self.hand: Hand | None = None
        self.error = None
        self.detect_error = None
        self.fps = self.detect_fps = 30.0
        self.overlay_active = False
        self.overlay_text = ""
        self.deadzone = 0.0

    def primary_hand(self, max_age_s: float = 0.4):
        return self.hand

    def hand_count(self):
        return 0 if self.hand is None else 1

    def show(self, cx: float, cy: float, xyz=None):
        self.hand = Hand(cx, cy, 0.3, 0.9, np.zeros((21, 2), np.float32), None if xyz is None else np.array(xyz))


class Rig:
    def __init__(self, tmp_path):
        self.cfg = Config(mock=True, calibration_dir=str(tmp_path))
        self.cams = {"wrist_left": FakeCamera(), "wrist_right": FakeCamera(), "head": FakeCamera("realsense")}
        self.arms = {"left": MockArm("left"), "right": MockArm("right")}
        self.ctl = Controller(self.cfg, self.arms, MockHead(), self.cams)
        self.ctl.connect_all()
        self.now = 100.0

    def run(self, seconds: float, keys: list[str] | None = None):
        for _ in range(int(seconds / DT)):
            self.now += DT
            for arm in self.arms.values():
                arm._last -= DT  # 让模拟臂的时间与测试时钟同步
            if keys is not None:
                self.ctl._keys, self.ctl._keys_time = set(keys), self.now
            self.ctl._tick(DT, self.now)

    def tip(self, arm: str):
        return self.ctl.kin.tip_and_pitch({j: self.ctl.arms[arm].pos[j] for j in ARM_JOINTS})

    def at_home(self, arm: str, tol: float = 1.5) -> bool:
        return all(abs(self.ctl.arms[arm].pos[j] - self.ctl.arms[arm].home[j]) < tol for j in ARM_JOINTS)


@pytest.fixture
def rig(tmp_path):
    return Rig(tmp_path)


def test_connect_does_not_move(rig):
    before = dict(rig.ctl.arms["left"].pos)
    rig.run(1.0)
    assert rig.ctl.arms["left"].pos == pytest.approx(before, abs=0.01)


def test_keyboard_mode_homes_then_moves_only_the_addressed_arm(rig):
    rig.ctl.set_mode("keyboard")
    rig.run(6.0)
    assert rig.at_home("left") and rig.at_home("right")

    left0, _ = rig.tip("left")
    right0, _ = rig.tip("right")
    rig.run(1.0, keys=["KeyW"])  # 左臂向前
    left1, pitch1 = rig.tip("left")
    right1, _ = rig.tip("right")
    assert left1[0] - left0[0] == pytest.approx(rig.cfg.keyboard.linear_speed * 1.0, abs=0.012)
    assert abs(left1[1] - left0[1]) < 0.005 and abs(left1[2] - left0[2]) < 0.005
    assert np.allclose(right0, right1, atol=1e-4)

    rig.run(1.0, keys=["KeyJ"])  # 右臂向左（+y）
    right2, _ = rig.tip("right")
    assert right2[1] - right1[1] > 0.03


def test_keys_expire_without_heartbeat(rig):
    rig.ctl.set_mode("keyboard")
    rig.run(6.0)
    rig.run(0.3, keys=["KeyR"])
    rig.run(1.0)  # 不再发心跳：按键应在超时后失效，手臂停住
    z_stopped = rig.tip("left")[0][2]
    rig.run(1.0)
    assert rig.tip("left")[0][2] == pytest.approx(z_stopped, abs=1e-4)


def test_keyboard_respects_workspace_and_midline(rig):
    rig.ctl.set_mode("keyboard")
    rig.run(6.0)
    rig.run(12.0, keys=["KeyW"])
    assert rig.tip("left")[0][0] <= rig.cfg.safety.x_range[1] + 0.01
    rig.run(12.0, keys=["KeyD"])  # 左臂向右，即朝另一只臂
    inward = rig.tip("left")[0][1] * rig.cfg.arms["left"].inward_y_sign
    assert inward <= rig.cfg.arms["left"].inward_limit + 0.01


def test_joint_speed_is_limited(rig):
    rig.ctl.set_mode("keyboard")
    rig.run(6.0)
    limit = rig.cfg.safety.max_joint_speed_deg_s * DT + 1e-6
    for _ in range(60):
        before = dict(rig.ctl.arms["left"].target)
        rig.run(DT, keys=["KeyW", "KeyA", "KeyR", "KeyQ"])
        after = rig.ctl.arms["left"].target
        assert all(abs(after[j] - before[j]) <= limit for j in ARM_JOINTS)


def test_estop_freezes_and_blocks_commands(rig):
    rig.ctl.set_mode("keyboard")
    rig.run(2.0)  # 正在回初始位的途中
    rig.ctl.set_estop(True)
    frozen = dict(rig.ctl.arms["left"].target)
    rig.run(1.0, keys=["KeyW"])
    assert rig.ctl.arms["left"].target == pytest.approx(frozen, abs=1e-6)
    with pytest.raises(RuntimeError):
        rig.ctl.set_mode("follow")
    with pytest.raises(RuntimeError):
        rig.ctl.go("home")
    rig.ctl.set_estop(False)
    assert rig.ctl.mode == "idle"
    rig.run(1.0)
    assert rig.ctl.arms["left"].target == pytest.approx(frozen, abs=1e-6)  # 解除后也不会自己动


def test_follow_turns_toward_hand_and_returns_home(rig):
    rig.ctl.set_mode("follow")
    rig.run(6.0)
    assert rig.at_home("left")
    yaw = lambda: np.arctan2(*rig.ctl.kin.approach_dir(  # noqa: E731
        {j: rig.ctl.arms["left"].pos[j] for j in ARM_JOINTS})[[1, 0]])
    yaw0, pitch0 = yaw(), rig.tip("left")[1]
    tip0 = rig.tip("left")[0]

    rig.cams["wrist_left"].show(0.9, 0.85)  # 手在画面右下
    rig.run(1.0)
    assert yaw() < yaw0 - 0.05  # 向右转
    assert rig.tip("left")[1] < pitch0 - 3  # 低头
    assert rig.at_home("right")  # 右臂没看到手，不动
    # 只转动不平移：肩部俯仰和肘部保持初始位
    for j in ("shoulder_lift", "elbow_flex", "wrist_roll"):
        assert rig.ctl.arms["left"].pos[j] == pytest.approx(rig.cfg.home[j], abs=0.5)
    assert rig.ctl.arms["left"].pos["gripper"] == pytest.approx(0.0, abs=0.5)  # 夹爪保持闭合
    assert np.linalg.norm(rig.tip("left")[0] - tip0) < 0.20

    rig.cams["wrist_left"].show(0.5, 0.5)  # 手回到中心：停止修正
    rig.run(0.5)
    held = dict(rig.ctl.arms["left"].target)
    rig.run(0.5)
    assert rig.ctl.arms["left"].target == pytest.approx(held, abs=0.05)

    rig.cams["wrist_left"].hand = None  # 手消失：短暂保持后回初始位
    rig.run(1.0)
    assert not rig.at_home("left")
    rig.run(6.0)
    assert rig.at_home("left")


def test_follow_offsets_are_bounded(rig):
    rig.ctl.set_mode("follow")
    rig.run(6.0)
    rig.cams["wrist_left"].show(1.0, 1.0)
    rig.run(20.0)
    rt = rig.ctl.arms["left"]
    f = rig.cfg.follow
    assert f.pan_range_deg[0] <= rt.pan_off <= f.pan_range_deg[1]
    assert f.tilt_range_deg[0] <= rt.tilt_off <= f.tilt_range_deg[1]


def _calibrate_identity(rig):
    """假设相机系与两臂基座系重合，便于直接给出基座系下的手位置。"""
    for name in ("left", "right"):
        rig.ctl.calibration.arms[name] = ArmExtrinsic(np.eye(4), 0.004, 8)


def test_fused_falls_back_to_wrist_follow_without_calibration(rig):
    rig.ctl.set_mode("fused")
    rig.run(6.0)
    rig.cams["head"].show(0.5, 0.5, xyz=[0.4, 0.0, 0.5])
    rig.cams["wrist_left"].show(0.9, 0.5)
    rig.run(1.0)
    assert rig.ctl.arms["left"].status.startswith("未标定")
    assert abs(rig.ctl.arms["left"].pan_off) > 1


def test_fused_guides_nearest_arm_and_keeps_distance(rig):
    _calibrate_identity(rig)
    rig.ctl.set_mode("fused")
    rig.run(6.0)
    hand = np.array([0.45, 0.10, 0.50])  # 满足假相机的深度范围检查：z 分量同时充当深度
    rig.cams["head"].show(0.5, 0.5, xyz=hand)
    rig.run(8.0)
    selected = rig.ctl._fused_arm
    other = "right" if selected == "left" else "left"
    tip, pitch = rig.tip(selected)
    home_tip, _ = rig.ctl.kin.tip_and_pitch(rig.cfg.home)
    assert np.linalg.norm(hand - tip) < np.linalg.norm(hand - home_tip) - 0.02  # 靠近了
    assert np.linalg.norm(hand - tip) >= rig.cfg.fused.standoff - 0.04  # 但保持距离
    assert pitch > 20  # 手在上方，夹爪抬头看向手
    assert rig.at_home(other)

    rig.cams["head"].hand = None
    rig.run(10.0)
    assert rig.at_home(selected)


def test_fused_retreats_when_hand_comes_close(rig):
    _calibrate_identity(rig)
    rig.ctl.set_mode("fused")
    rig.run(6.0)
    rig.cams["head"].show(0.5, 0.5, xyz=[0.62, 0.0, 0.30])
    rig.run(8.0)
    far_reach = np.linalg.norm(rig.tip(rig.ctl._fused_arm)[0] - rig.ctl.shoulder)
    rig.cams["head"].show(0.5, 0.5, xyz=[0.42, 0.0, 0.26])
    rig.run(8.0)
    near_reach = np.linalg.norm(rig.tip(rig.ctl._fused_arm)[0] - rig.ctl.shoulder)
    assert near_reach < far_reach - 0.03


def test_stall_makes_arm_yield(rig):
    rig.ctl.set_mode("keyboard")
    rig.run(6.0)
    arm = rig.arms["left"]
    arm._pos["shoulder_pan"] += 40.0  # 模拟被人推开
    arm._target["shoulder_pan"] = arm._pos["shoulder_pan"]
    rig.run(DT)
    rt = rig.ctl.arms["left"]
    assert "阻挡" in rt.status
    assert rt.target["shoulder_pan"] == pytest.approx(arm._pos["shoulder_pan"], abs=1.0)


def test_rest_then_release_torque(rig):
    rig.ctl.set_mode("keyboard")
    rig.run(6.0)
    rig.ctl.go("rest", release=True)
    rig.run(10.0)
    for arm in rig.arms.values():
        assert not arm.torque_enabled
    assert rig.ctl.arms["left"].pos["shoulder_lift"] == pytest.approx(rig.cfg.rest["shoulder_lift"], abs=1.0)


def test_snapshot_is_json_serialisable(rig):
    import json

    rig.run(0.2)
    snap = rig.ctl.snapshot()
    json.dumps(snap)
    assert set(snap["arms"]) == {"left", "right"}
    assert snap["mode"] == "idle"
