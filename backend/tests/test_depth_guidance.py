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
