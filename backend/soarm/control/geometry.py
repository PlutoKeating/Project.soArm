"""腕部相机与手臂之间的几何换算。纯函数，控制器和离线工具共用。

约定：腕部相机装在夹爪上，光轴沿夹爪指向；初始位的腕部旋转已让画面是正的，
所以画面的“上”就是世界的“上”在垂直于光轴平面内的投影。
"""

from __future__ import annotations

import numpy as np

from ..kinematics import ArmKinematics

_UP = np.array([0.0, 0.0, 1.0])


NO_OFFSET = (0.0, 0.0)


def camera_axes(
    kin: ArmKinematics, joints: dict[str, float], mount_offset_deg: tuple[float, float] = NO_OFFSET
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """腕部相机在基座系下的 (位置, 光轴, 画面右方向, 画面上方向)。

    mount_offset_deg 是相机光轴相对夹爪指向的安装偏角 (偏右, 偏上)，单位度。实测相机是从
    夹爪上方斜着往下看的，光轴比夹爪指向低约 20°；不计这个偏角，换算出的方向会整体偏掉。
    """
    tip, origin = kin._frames(joints)
    forward = tip[:3, 3] - origin
    forward /= np.linalg.norm(forward)
    up = _UP - (_UP @ forward) * forward
    norm = np.linalg.norm(up)
    up = up / norm if norm > 1e-6 else np.array([1.0, 0.0, 0.0])
    right = np.cross(forward, up)  # x 向前、z 向上时为 -y，即向右
    yaw, pitch = np.deg2rad(mount_offset_deg[0]), np.deg2rad(mount_offset_deg[1])
    if yaw or pitch:
        forward, right = forward * np.cos(yaw) + right * np.sin(yaw), right * np.cos(yaw) - forward * np.sin(yaw)
        forward, up = forward * np.cos(pitch) + up * np.sin(pitch), up * np.cos(pitch) - forward * np.sin(pitch)
    return origin, forward, right, up


def wrist_ray(
    kin: ArmKinematics,
    joints: dict[str, float],
    cx: float,
    cy: float,
    hfov_deg: float,
    aspect: float,
    mount_offset_deg: tuple[float, float] = NO_OFFSET,
) -> tuple[np.ndarray, np.ndarray]:
    """腕部相机画面上归一化坐标 (cx, cy) 对应的视线：返回基座系下的 (起点, 单位方向)。

    aspect 是画面高宽比。视场角是估计值，所以越靠画面中心的点换算出的方向越准。
    """
    origin, forward, right, up = camera_axes(kin, joints, mount_offset_deg)
    half_w = np.tan(np.deg2rad(hfov_deg) / 2.0)
    nx, ny = (cx - 0.5) * 2.0, (cy - 0.5) * 2.0
    direction = forward + nx * half_w * right - ny * half_w * aspect * up
    return origin, direction / np.linalg.norm(direction)


def _az_el(v: np.ndarray) -> tuple[float, float]:
    return float(np.arctan2(v[1], v[0])), float(np.arctan2(v[2], np.hypot(v[0], v[1])))


def aim_offsets(
    kin: ArmKinematics,
    home: dict[str, float],
    point: np.ndarray,
    pan_dir: float,
    tilt_dir: float,
    mount_offset_deg: tuple[float, float] = NO_OFFSET,
) -> np.ndarray:
    """让腕部相机光轴指向基座系下某一点，所需的 (肩部旋转, 腕部俯仰) 相对初始位的偏转量（度）。

    相机位置会随偏转变化，所以迭代几次。pan_dir / tilt_dir 是这两个关节正方向
    对方位角 / 俯仰角的符号。
    """
    offsets = np.zeros(2)
    for _ in range(4):
        joints = dict(home)
        joints["shoulder_pan"] += offsets[0]
        joints["wrist_flex"] += offsets[1]
        origin, forward, _, _ = camera_axes(kin, joints, mount_offset_deg)
        az_t, el_t = _az_el(point - origin)
        az, el = _az_el(forward)
        d_az = (az_t - az + np.pi) % (2 * np.pi) - np.pi
        offsets = offsets + np.rad2deg([d_az * pan_dir, (el_t - el) * tilt_dir])
    return offsets
