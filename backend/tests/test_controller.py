"""用模拟臂和假相机逐拍驱动控制器，验证三种模式和安全逻辑。不需要任何硬件。"""

import time
from pathlib import Path

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
        self.detect_enabled = True
        self.fps = self.detect_fps = 30.0
        self.read_ms = self.detect_ms = 0.0
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
            for cam in self.cams.values():
                if cam.hand is not None:
                    cam.hand.t = self.now  # 模拟每拍都有新的一帧
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
    pushed = arm._pos["shoulder_pan"] + 40.0
    rt = rig.ctl.arms["left"]
    for i in range(15):  # 模拟被人持续顶住约半秒
        arm._pos["shoulder_pan"] = arm._target["shoulder_pan"] = pushed
        rig.run(DT)
        if i == 2:
            assert "阻挡" not in rt.status  # 瞬时偏差不算受阻
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


def test_disabled_arm_holds_in_every_mode(rig):
    rig.ctl.set_arm_enabled("left", False)
    before = dict(rig.ctl.arms["left"].pos)
    rig.ctl.set_mode("follow")
    rig.cams["wrist_left"].show(0.9, 0.9)
    rig.run(6.0)
    assert rig.ctl.arms["left"].pos == pytest.approx(before, abs=0.01)
    assert rig.at_home("right")
    rig.ctl.go("home")
    rig.run(3.0)
    assert rig.ctl.arms["left"].pos == pytest.approx(before, abs=0.01)
    rig.ctl.set_arm_enabled("left", True)  # 跟随模式下重新启用会先回初始位
    rig.ctl.set_mode("follow")
    rig.cams["wrist_left"].hand = None
    rig.run(6.0)
    assert rig.at_home("left")


def test_pan_never_crosses_toward_the_mast(rig):
    """中柱避让：左臂不超过前偏右 30°（+pan），右臂不超过前偏左 30°（-pan）。"""
    rig.ctl.set_mode("follow")
    rig.run(6.0)
    rig.cams["wrist_left"].show(1.0, 0.5)   # 手一直在左臂画面最右
    rig.cams["wrist_right"].show(0.0, 0.5)  # 手一直在右臂画面最左
    rig.run(15.0)
    assert 20.0 < rig.ctl.arms["left"].pos["shoulder_pan"] <= 30.0 + 1e-6
    assert -30.0 - 1e-6 <= rig.ctl.arms["right"].pos["shoulder_pan"] < -20.0
    rig.ctl.set_mode("keyboard")
    rig.run(6.0)
    rig.run(15.0, keys=["KeyD", "KeyJ"])  # 左臂向右、右臂向左
    assert rig.ctl.arms["left"].target["shoulder_pan"] <= 30.0 + 1e-6
    assert rig.ctl.arms["right"].target["shoulder_pan"] >= -30.0 - 1e-6


def _show(cam, cx, cy, size=0.4, score=0.9):
    cam.hand = Hand(cx, cy, size, score, np.zeros((21, 2), np.float32), None)


def test_only_one_arm_tracks_and_the_better_view_wins(rig):
    rig.cfg.follow.single_tracker = True
    rig.ctl.set_mode("follow")
    rig.run(6.0)
    # 两只臂都看到手：左臂画面里手居中偏一点，右臂画面里手在边角且很小
    _show(rig.cams["wrist_left"], 0.62, 0.5)
    _show(rig.cams["wrist_right"], 0.95, 0.9, size=0.15)
    rig.run(1.0)
    assert rig.ctl._follow_arm == "left"
    assert abs(rig.ctl.arms["left"].pan_off) > 1
    assert rig.at_home("right")  # 右臂看到了手也不动
    assert "让位" in rig.ctl.arms["right"].status
    snap = rig.ctl.snapshot()
    assert snap["arms"]["left"]["fused_selected"] and not snap["arms"]["right"]["fused_selected"]


def test_tracker_switches_only_when_other_view_is_clearly_better(rig):
    rig.cfg.follow.single_tracker = True
    rig.ctl.set_mode("follow")
    rig.run(6.0)
    _show(rig.cams["wrist_left"], 0.6, 0.5)
    rig.run(0.5)
    assert rig.ctl._follow_arm == "left"
    # 右臂也看到了，质量略好一点点：不换
    _show(rig.cams["wrist_left"], 0.6, 0.5, score=0.85)
    _show(rig.cams["wrist_right"], 0.55, 0.5, score=0.9)
    rig.run(2.0)
    assert rig.ctl._follow_arm == "left"
    # 左臂视野明显变差（手到了边角），右臂居中：短暂领先不换，持续领先才换
    _show(rig.cams["wrist_left"], 0.95, 0.92, size=0.2, score=0.6)
    _show(rig.cams["wrist_right"], 0.5, 0.5)
    rig.run(0.3)
    assert rig.ctl._follow_arm == "left"
    rig.run(1.0)
    assert rig.ctl._follow_arm == "right"
    # 换臂后原来的臂回初始位
    rig.run(8.0)
    assert rig.at_home("left")


def test_tracker_hands_over_after_grace_when_current_loses_hand(rig):
    rig.cfg.follow.single_tracker = True
    rig.ctl.set_mode("follow")
    rig.run(6.0)
    _show(rig.cams["wrist_left"], 0.6, 0.5)
    rig.run(0.5)
    rig.cams["wrist_left"].hand = None
    _show(rig.cams["wrist_right"], 0.9, 0.9, size=0.2, score=0.6)  # 视野一般，但只有它看得到
    rig.run(0.2)
    assert rig.ctl._follow_arm == "left"  # 漏检一两帧不移交
    _show(rig.cams["wrist_left"], 0.6, 0.5)
    rig.run(0.2)
    assert rig.ctl._follow_arm == "left"  # 手回来了，继续由它跟
    rig.cams["wrist_left"].hand = None
    rig.run(1.0)
    assert rig.ctl._follow_arm == "right"  # 确实丢了才移交
    rig.cams["wrist_right"].hand = None
    rig.run(1.0)
    assert rig.ctl._follow_arm is None


def test_disabled_arm_is_never_chosen_as_tracker(rig):
    rig.cfg.follow.single_tracker = True
    rig.ctl.set_arm_enabled("left", False)
    rig.ctl.set_mode("follow")
    rig.run(6.0)
    _show(rig.cams["wrist_left"], 0.5, 0.5)
    _show(rig.cams["wrist_right"], 0.9, 0.9, size=0.2, score=0.6)
    rig.run(1.0)
    assert rig.ctl._follow_arm == "right"


def _sweep(rig, speed_deg_s: float, seconds: float, latency_s: float = 0.1):
    """闭环模拟：手以恒定角速度横向移动，相机按“手与手臂朝向之差”成像，并带固定延迟。

    返回 (最大滞后角, 结束时手的方向, 结束时手臂的偏转量)。全部在偏转量空间里计算。
    """
    ctl, rt, cam = rig.ctl, rig.ctl.arms["left"], rig.cams["wrist_left"]
    f = rig.cfg.follow
    sign = ctl._pan_dir * rt.cfg.follow_pan_sign
    half_fov = cam.cfg.hfov_deg / 2.0
    hand_dir, worst, frames = 0.0, 0.0, []
    for _ in range(int(seconds / DT)):
        hand_dir -= speed_deg_s * DT  # 朝偏转量减小的方向移动，避开左臂向右的中柱限位
        offset = rt.pos["shoulder_pan"] - rt.home["shoulder_pan"]
        frames.append((rig.now, 0.5 - (hand_dir - offset) * sign / half_fov / 2.0))
        seen = [fr for fr in frames if fr[0] <= rig.now - latency_s]
        if seen and 0.0 <= seen[-1][1] <= 1.0:
            cam.hand = Hand(seen[-1][1], 0.5, 0.4, 0.9, np.zeros((21, 2), np.float32), None, seen[-1][0])
        else:
            cam.hand = None
        # 这里自己控制观测的时间戳，所以不用 rig.run（它会把时间戳改成当前时刻）
        rig.now += DT
        for arm in rig.arms.values():
            arm._last -= DT
        ctl._tick(DT, rig.now)
        worst = max(worst, abs(hand_dir - (rt.pos["shoulder_pan"] - rt.home["shoulder_pan"])))
    return worst, hand_dir, rt.pos["shoulder_pan"] - rt.home["shoulder_pan"]


def test_follow_keeps_up_with_a_fast_hand_despite_latency(rig):
    rig.cfg.follow.aim_scale = 1.0  # 模拟里视场角是准确的
    rig.ctl.set_mode("follow")
    rig.run(6.0)
    half_fov = rig.cams["wrist_left"].cfg.hfov_deg / 2.0
    worst, hand_dir, offset = _sweep(rig, speed_deg_s=60.0, seconds=0.6)
    assert worst < half_fov * 0.5  # 手始终留在画面中部，不会被甩出视野
    assert abs(hand_dir - offset) < 10.0


def test_follow_coasts_in_the_same_direction_after_losing_the_hand(rig):
    rig.cfg.follow.aim_scale = 1.0
    rig.ctl.set_mode("follow")
    rig.run(6.0)
    _sweep(rig, speed_deg_s=60.0, seconds=0.5)
    rt = rig.ctl.arms["left"]
    at_loss = rt.pan_off
    rig.cams["wrist_left"].hand = None
    rig.run(0.3)
    assert "继续追" in rt.status
    assert rt.pan_off < at_loss - 3.0  # 丢失后沿原方向（偏转量减小）继续转
    rig.run(8.0)
    assert rig.at_home("left")  # 一直找不到就回初始位


def test_both_arms_follow_their_own_hand_independently(rig):
    """默认：两只臂各跟各的，同时在跟，互不让位。"""
    rig.ctl.set_mode("follow")
    rig.run(6.0)
    _show(rig.cams["wrist_left"], 0.2, 0.5)   # 左臂画面里手在左侧
    _show(rig.cams["wrist_right"], 0.8, 0.3)  # 右臂画面里手在右上
    rig.run(1.0)
    left, right = rig.ctl.arms["left"], rig.ctl.arms["right"]
    assert abs(left.pan_off) > 3 and abs(right.pan_off) > 3
    assert np.sign(left.pan_off) != np.sign(right.pan_off)  # 各自朝自己那只手转，方向相反
    assert "让位" not in left.status and "让位" not in right.status
    snap = rig.ctl.snapshot()
    assert snap["arms"]["left"]["fused_selected"] and snap["arms"]["right"]["fused_selected"]
    # 一只手消失只影响对应的那只臂
    rig.cams["wrist_left"].hand = None
    rig.run(8.0)
    assert rig.at_home("left")
    assert abs(rig.ctl.arms["right"].pan_off) > 3
    assert not rig.ctl.snapshot()["arms"]["left"]["fused_selected"]


def test_lend_arm_releases_it_without_dropping_torque_and_restore_takes_it_back(rig, monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path))
    rig.ctl.set_mode("follow")
    rig.run(6.0)
    right = rig.arms["right"]
    released = []
    monkeypatch.setattr(right, "disconnect", lambda: (released.append(True), setattr(right, "connected", False)))
    rig.ctl.lend_arm("right")
    assert released and not right.connected
    assert right.torque_enabled  # 力矩没掉
    snap = rig.ctl.snapshot()
    assert snap["arms"]["right"]["lent"] and not snap["arms"]["left"]["lent"]
    # 借出期间：另一只臂照常跟随；“重新连接设备”不会把借出的臂抢回来
    _show(rig.cams["wrist_left"], 0.2, 0.5)
    rig.run(1.0)
    assert abs(rig.ctl.arms["left"].pan_off) > 3
    rig.ctl.connect_all()
    assert not right.connected
    # 恢复：接管并回到跟随
    result = rig.ctl.restore_all()
    assert result["restored"] == ["right"] and not result["failed"]
    assert right.connected and not rig.ctl.arms["right"].lent
    rig.cams["wrist_left"].hand = None
    rig.run(8.0)
    assert rig.at_home("right") and rig.ctl.mode == "follow"


def test_restore_evicts_the_registered_lab_process(monkeypatch, tmp_path):
    import subprocess
    import sys

    from soarm import lab

    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path))
    # 一个登记在案、收到 SIGTERM 会走清理流程的“开发程序”
    code = (
        "import time, sys; sys.path.insert(0, %r); from soarm import lab; lab.claim()\n"
        "try:\n    print('ready', flush=True); time.sleep(60)\n"
        "except KeyboardInterrupt:\n    lab.release(); print('cleaned', flush=True)\n"
    ) % str(Path(lab.__file__).resolve().parents[1])
    proc = subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE, text=True)
    assert proc.stdout.readline().strip() == "ready"
    t0 = time.monotonic()
    note = lab.evict()
    assert "已退出" in note and time.monotonic() - t0 < 3.0
    assert proc.wait(timeout=3) == 0 and proc.stdout.read().strip() == "cleaned"
    assert not lab.pidfile().exists()
    assert "没有" in lab.evict()  # 再调一次：没有可终止的
