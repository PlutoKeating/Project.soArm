"""深度相机引导相关的几何、外参自动求解。全部用合成数据，不接触硬件。"""

import numpy as np
import pytest

from soarm.calib import BearingSample, solve_from_bearings
from soarm.config import Config
from soarm.control.geometry import aim_offsets, camera_axes, wrist_ray
from soarm.kinematics import ARM_JOINTS, ArmKinematics

KIN = ArmKinematics()
HOME = {j: Config().home[j] for j in ARM_JOINTS}
HFOV, ASPECT = 90.0, 720 / 1280


def _true_extrinsic():
    """一个接近实物的外参：相机在臂的右后上方，朝前并低头 35°。"""
    pitch = np.deg2rad(35.0)
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    R = Ry @ np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, [-0.05, -0.20, 0.42]
    return T


def _project(joints, point):
    """基座系下的点在腕部相机画面上的归一化坐标；wrist_ray 的逆运算。"""
    origin, forward, right, up = camera_axes(KIN, joints)
    v = point - origin
    depth = v @ forward
    half_w = np.tan(np.deg2rad(HFOV) / 2)
    return 0.5 + (v @ right) / depth / half_w / 2, 0.5 - (v @ up) / depth / (half_w * ASPECT) / 2


def _samples(T, n=60, noise_px=0.0, noise_m=0.0, seed=0):
    rng = np.random.default_rng(seed)
    T_inv = np.linalg.inv(T)
    out = []
    while len(out) < n:
        hand = np.array([rng.uniform(0.35, 0.75), rng.uniform(-0.25, 0.25), rng.uniform(0.05, 0.45)])
        joints = dict(HOME)
        joints["shoulder_pan"] += rng.uniform(-25, 25)
        joints["wrist_flex"] += rng.uniform(-15, 20)
        cx, cy = _project(joints, hand)
        if not (0.2 < cx < 0.8 and 0.2 < cy < 0.8):
            continue  # 只用画面中部的观测，和实际采集的筛选一致
        cx, cy = cx + rng.normal(0, noise_px), cy + rng.normal(0, noise_px)
        origin, direction = wrist_ray(KIN, joints, cx, cy, HFOV, ASPECT)
        cam = T_inv[:3, :3] @ hand + T_inv[:3, 3] + rng.normal(0, noise_m, 3)
        out.append(BearingSample(origin, direction, cam))
    return out


def test_wrist_ray_points_forward_at_image_centre_and_follows_pixels():
    origin, centre = wrist_ray(KIN, HOME, 0.5, 0.5, HFOV, ASPECT)
    assert centre[0] > 0.95
    _, right = wrist_ray(KIN, HOME, 0.9, 0.5, HFOV, ASPECT)
    _, down = wrist_ray(KIN, HOME, 0.5, 0.9, HFOV, ASPECT)
    assert right[1] < centre[1] - 0.2  # 画面右侧 = 基座系 -y
    assert down[2] < centre[2] - 0.1  # 画面下方 = 向下
    # 投影与视线互为逆运算
    point = origin + 0.4 * right
    assert _project(HOME, point) == pytest.approx((0.9, 0.5), abs=1e-6)


def test_aim_offsets_put_the_point_on_the_optical_axis():
    ctl_pan_dir, ctl_tilt_dir = -1.0, -1.0  # 与控制器在初始位处求得的一致，见下面的断言
    base = dict(HOME)
    _, f0, _, _ = camera_axes(KIN, base)
    _, f1, _, _ = camera_axes(KIN, {**base, "shoulder_pan": base["shoulder_pan"] + 5})
    assert np.sign(np.arctan2(f1[1], f1[0]) - np.arctan2(f0[1], f0[0])) == ctl_pan_dir
    _, f2, _, _ = camera_axes(KIN, {**base, "wrist_flex": base["wrist_flex"] + 5})
    assert np.sign(f2[2] - f0[2]) == ctl_tilt_dir
    for point in ([0.6, 0.2, 0.35], [0.5, -0.25, 0.1], [0.8, 0.0, 0.5]):
        off = aim_offsets(KIN, HOME, np.array(point), ctl_pan_dir, ctl_tilt_dir)
        joints = dict(HOME)
        joints["shoulder_pan"] += off[0]
        joints["wrist_flex"] += off[1]
        assert _project(joints, np.array(point)) == pytest.approx((0.5, 0.5), abs=0.01)


def test_bearing_calibration_recovers_extrinsic_exactly_without_noise():
    T = _true_extrinsic()
    solved, rms, used = solve_from_bearings(_samples(T))
    assert np.allclose(solved, T, atol=2e-3)
    assert rms < 1e-3 and used == 60


def test_bearing_calibration_is_usable_with_realistic_noise():
    T = _true_extrinsic()
    # 腕部画面上 2% 的像素噪声（约 1.8°），深度测量 1 厘米噪声
    solved, rms, _ = solve_from_bearings(_samples(T, n=120, noise_px=0.02, noise_m=0.01, seed=3))
    rng = np.random.default_rng(9)
    errors = []
    for _ in range(200):
        cam = np.linalg.inv(T) @ np.array([rng.uniform(0.35, 0.75), rng.uniform(-0.25, 0.25), rng.uniform(0.05, 0.45), 1])
        errors.append(np.linalg.norm((solved @ cam)[:3] - (T @ cam)[:3]))
    assert np.median(errors) < 0.03  # 工作区内定位误差中位数 3 厘米以内，足够把手带进腕部视野
    assert rms < 0.03


def test_bearing_calibration_rejects_outliers():
    T = _true_extrinsic()
    samples = _samples(T, n=80, seed=5)
    rng = np.random.default_rng(1)
    for s in samples[:8]:  # 10% 的样本其实是另一只手
        s.cam_point = s.cam_point + rng.uniform(-0.3, 0.3, 3)
    solved, _, used = solve_from_bearings(samples)
    assert used <= 76
    assert np.allclose(solved[:3, 3], T[:3, 3], atol=0.02)


def test_bearing_calibration_refuses_too_few_or_degenerate_samples():
    T = _true_extrinsic()
    with pytest.raises(ValueError):
        solve_from_bearings(_samples(T, n=5))
    flat = _samples(T, n=40)
    for s in flat:
        s.cam_point = np.array([s.cam_point[0], s.cam_point[1], 0.5])
    with pytest.raises(ValueError):
        solve_from_bearings(flat)


# ---------------------------------------------------------------- 控制器级：指向式深度引导
from soarm.calib import ArmExtrinsic  # noqa: E402
from soarm.perception.hand import Hand  # noqa: E402
from test_controller import DT, Rig  # noqa: E402


class World:
    """一只在空间里的手，被头部深度相机和两个腕部相机按真实几何成像。"""

    def __init__(self, rig, T_left, T_right, calibrated=True):
        self.rig, self.T = rig, {"left": T_left, "right": T_right}
        self.hand_in = "left"  # 手的位置用哪只臂的基座系表示
        self.point = None
        self.wrist_blind = set()  # 这些臂的腕部相机“看不到”，用来测试纯深度引导
        if calibrated:
            for name, T in self.T.items():
                rig.ctl.calibration.arms[name] = ArmExtrinsic(T, 0.01, 50)

    def _in_base(self, name):
        cam = np.linalg.inv(self.T[self.hand_in]) @ np.append(self.point, 1.0)
        return (self.T[name] @ cam)[:3], cam[:3]

    def step(self, seconds):
        rig = self.rig
        for _ in range(int(seconds / DT)):
            rig.now += DT
            for arm in rig.arms.values():
                arm._last -= DT
            head = rig.cams["head"]
            if self.point is None:
                head.hand = None
            else:
                _, cam_pt = self._in_base("left")
                head.hand = Hand(0.5, 0.5, 0.2, 0.9, np.zeros((21, 2), np.float32), cam_pt, rig.now)
            for name in ("left", "right"):
                cam = rig.cams[f"wrist_{name}"]
                cam.hand = None
                if self.point is None or name in self.wrist_blind:
                    continue
                rt = rig.ctl.arms[name]
                joints = {j: rt.pos[j] for j in ARM_JOINTS}
                origin, forward, right, up = camera_axes(KIN, joints)
                v = self._in_base(name)[0] - origin
                depth = v @ forward
                if depth <= 0.05:
                    continue
                half_w = np.tan(np.deg2rad(cam.cfg.hfov_deg) / 2)
                aspect = cam.cfg.height / cam.cfg.width
                cx = 0.5 + (v @ right) / depth / half_w / 2
                cy = 0.5 - (v @ up) / depth / (half_w * aspect) / 2
                if 0.0 <= cx <= 1.0 and 0.0 <= cy <= 1.0:
                    cam.hand = Hand(cx, cy, 0.4, 0.9, np.zeros((21, 2), np.float32), None, rig.now)
            rig.ctl._tick(DT, rig.now)

    def pixel(self, name):
        hand = self.rig.cams[f"wrist_{name}"].hand
        return None if hand is None else (hand.cx, hand.cy)


def _two_arm_extrinsics():
    """两只臂的基座左右相距 0.4 米，相机在中间上方。"""
    T_left = _true_extrinsic()
    T_right = T_left.copy()
    T_right[1, 3] += 0.40  # 同一台相机，在右臂基座系里更靠左
    return T_left, T_right


@pytest.fixture
def world(tmp_path):
    rig = Rig(tmp_path)
    for cam in ("wrist_left", "wrist_right"):
        rig.cams[cam].cfg.width, rig.cams[cam].cfg.height, rig.cams[cam].cfg.hfov_deg = 1280, 720, 90.0
    rig.cfg.follow.aim_scale = 1.0
    w = World(rig, *_two_arm_extrinsics())
    rig.ctl.set_mode("fused")
    rig.run(6.0)
    assert rig.at_home("left") and rig.at_home("right")
    return w


def test_depth_cue_points_the_arm_at_a_hand_its_wrist_camera_cannot_see(world):
    rig = world.rig
    world.wrist_blind = {"left", "right"}  # 只有深度相机看得到
    world.hand_in, world.point = "left", np.array([0.55, 0.30, 0.30])  # 左臂的左前方
    world.step(3.0)
    assert rig.ctl._cue_arm == "left"
    assert "深度引导" in rig.ctl.arms["left"].status
    # 腕部相机光轴已经对准了手：把“看不到”去掉后，手应落在画面中心附近
    world.wrist_blind = set()
    world.step(DT)
    cx, cy = world.pixel("left")
    assert abs(cx - 0.5) < 0.08 and abs(cy - 0.5) < 0.10
    assert rig.at_home("right")  # 另一只臂不动


def test_depth_cue_picks_the_arm_that_needs_less_turning(world):
    rig = world.rig
    world.wrist_blind = {"left", "right"}
    world.hand_in, world.point = "right", np.array([0.55, -0.25, 0.25])  # 右臂的右前方
    world.step(3.0)
    assert rig.ctl._cue_arm == "right"
    assert rig.at_home("left")


def test_wrist_camera_takes_over_once_it_sees_the_hand(world):
    rig = world.rig
    world.hand_in, world.point = "left", np.array([0.55, 0.30, 0.30])
    world.step(3.0)
    assert rig.ctl._cue_arm is None  # 腕部相机自己看到了，不再需要引导
    assert rig.ctl.arms["left"].status in ("跟随中", "已对准")
    cx, cy = world.pixel("left")
    assert abs(cx - 0.5) < 0.08 and abs(cy - 0.5) < 0.10


def test_arms_return_home_when_hand_disappears(world):
    rig = world.rig
    world.hand_in, world.point = "left", np.array([0.55, 0.30, 0.30])
    world.step(3.0)
    world.point = None
    world.step(8.0)
    assert rig.at_home("left") and rig.at_home("right")


def test_uncalibrated_arm_gets_no_depth_cue(tmp_path):
    rig = Rig(tmp_path)
    w = World(rig, *_two_arm_extrinsics(), calibrated=False)
    rig.ctl.set_mode("fused")
    rig.run(6.0)
    w.wrist_blind = {"left", "right"}
    w.hand_in, w.point = "left", np.array([0.55, 0.30, 0.30])
    w.step(2.0)
    assert rig.ctl._cue_arm is None and rig.at_home("left")


def test_auto_calibration_collects_pairs_while_tracking_and_recovers_extrinsic(tmp_path):
    """没有外参时，只靠腕部跟随过程中自动积累的配对样本求出外参。"""
    rig = Rig(tmp_path)
    for cam in ("wrist_left", "wrist_right"):
        rig.cams[cam].cfg.width, rig.cams[cam].cfg.height, rig.cams[cam].cfg.hfov_deg = 1280, 720, 90.0
    rig.cfg.follow.aim_scale = 1.0
    T_left, T_right = _two_arm_extrinsics()
    w = World(rig, T_left, T_right, calibrated=False)
    rig.ctl.set_mode("follow")  # 普通的腕部跟随模式下也在采集
    rig.run(6.0)
    rng = np.random.default_rng(4)
    w.hand_in = "left"
    w.wrist_blind = {"right"}  # 这里只考察左臂；否则手移到中间时右臂也可能拿到跟随权
    for _ in range(45):  # 手在左臂前方慢慢换位置
        w.point = np.array([rng.uniform(0.40, 0.75), rng.uniform(-0.05, 0.30), rng.uniform(0.10, 0.40)])
        w.step(1.2)
    n = len(rig.ctl.auto_samples["left"])
    assert n >= 20
    assert len(rig.ctl.auto_samples["right"]) == 0
    result = rig.ctl.calib_auto_solve("left")
    solved = rig.ctl.calibration.arms["left"].T_base_cam
    assert result["rms"] < 0.02
    assert np.allclose(solved[:3, 3], T_left[:3, 3], atol=0.03)
    assert (tmp_path / "extrinsics.yaml").exists()
    with pytest.raises(ValueError):
        rig.ctl.calib_auto_solve("right")


def test_depth_cue_guides_only_the_arm_that_cannot_see_while_the_other_keeps_following(world):
    """两臂独立跟随时：一只臂的腕部相机在跟自己的手，另一只看不到，深度引导只带后者。"""
    rig = world.rig
    world.wrist_blind = {"right"}
    world.hand_in, world.point = "left", np.array([0.55, 0.05, 0.30])
    world.step(3.0)
    assert rig.ctl.arms["left"].status in ("跟随中", "已对准")
    assert rig.ctl._cue_arm == "right"
    assert "深度引导" in rig.ctl.arms["right"].status
    snap = rig.ctl.snapshot()
    assert snap["arms"]["left"]["fused_selected"] and snap["arms"]["right"]["fused_selected"]
