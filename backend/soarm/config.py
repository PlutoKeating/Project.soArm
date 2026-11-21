"""配置：默认值写在 dataclass 里，config/config.yaml 只需覆盖与本机有关的项。"""

from __future__ import annotations

import dataclasses
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = REPO_ROOT / "config" / "config.yaml"


@dataclass
class ServerConfig:
    host: str = "127.0.0.1"
    port: int = 8000


@dataclass
class ArmConfig:
    port: str = ""  # 建议用 /dev/serial/by-id/... 的稳定路径
    calibration_id: str = ""  # LeRobot 校准文件的 id，默认与臂名相同
    camera: str = ""  # 对应腕部相机在 cameras 里的名字
    # 腕部相机画面与关节方向的对应关系，取 +1 或 -1，上真机后各按一次键确认
    follow_pan_sign: float = 1.0
    follow_tilt_sign: float = 1.0
    # 夹爪参考点朝另一只臂方向最多能伸多远（米），防止两臂越过中线相撞
    inward_limit: float = 0.10
    # 在本臂基座系里，“朝向另一只臂”是 +y 还是 -y
    inward_y_sign: float = 1.0
    # 额外的关节限位（度），与 URDF 限位、舵机行程限位取交集。
    # 用于避让两臂之间的相机中柱：肩部旋转正方向是向右转
    joint_limits: dict[str, tuple[float, float]] = field(default_factory=dict)
    # 覆盖全局 home 里的个别关节。主要用于 wrist_roll：要让腕部相机画面是正的
    home: dict[str, float] = field(default_factory=dict)


@dataclass
class HeadMotorConfig:
    id: int = 1
    center: int = 2048  # 预设“朝前”姿态的原始刻度（0-4095）
    min: int = 1500
    max: int = 2600


@dataclass
class HeadConfig:
    port: str = ""
    pan: HeadMotorConfig = field(default_factory=lambda: HeadMotorConfig(id=1))
    tilt: HeadMotorConfig = field(default_factory=lambda: HeadMotorConfig(id=2))
    # 键盘调整云台的速度（度/秒）
    speed_deg_s: float = 25.0


@dataclass
class CameraConfig:
    enabled: bool = True  # 设为 false 则完全不打开这路相机，把设备留给别的程序
    type: str = "opencv"  # opencv | realsense | remote（读另一个控制台的检测结果，device 写 "http://主机:端口#相机名"）
    device: str = ""  # opencv: /dev/v4l/by-path/...-usbv2-... 或整数索引；realsense: 序列号，留空取第一台
    width: int = 640
    height: int = 480
    fps: int = 30
    rotate: int = 0  # 0 / 90 / 180 / 270，顺时针
    hfov_deg: float = 70.0  # 水平视场角，仅腕部对准用于把像素误差换算成角度
    detect: bool = True  # 是否在此相机上跑手部检测


@dataclass
class SafetyConfig:
    """展会场景的保守默认值。"""

    max_joint_speed_deg_s: float = 180.0  # 任何模式下单关节目标的变化速率上限
    home_speed_deg_s: float = 25.0  # 回初始位的速度
    max_torque_limit: int = 1000  # Feetech Torque_Limit，满量程 1000。5 V 供电下调低会托不住手臂
    acceleration: int = 150  # Feetech Acceleration 寄存器（单位约 8.8°/s²），越小越柔但越跟不上
    # 夹爪参考点在本臂基座系下允许的范围（米）
    x_range: tuple[float, float] = (0.10, 0.38)
    y_range: tuple[float, float] = (-0.24, 0.24)
    z_range: tuple[float, float] = (0.06, 0.40)
    pitch_range_deg: tuple[float, float] = (-70.0, 60.0)
    ik_pos_tolerance: float = 0.012  # IK 位置残差超过它就拒绝这一步
    key_timeout_s: float = 0.35  # 这么久没收到按键心跳就视为全部松开
    joint_limit_margin_deg: float = 5.0
    # 关节实际位置与目标偏差过大（被人挡住/顶住）时暂停该臂
    stall_error_deg: float = 30.0
    stall_hold_s: float = 0.3  # 偏差持续这么久才算受阻，避免快速运动时的瞬时滞后误触发


@dataclass
class KeyboardConfig:
    linear_speed: float = 0.06  # 米/秒
    pitch_speed_deg_s: float = 35.0
    roll_speed_deg_s: float = 60.0
    gripper_speed: float = 70.0  # 夹爪开合度 0-100，每秒变化量


@dataclass
class FollowConfig:
    """第 2 步：腕部相机只对准、不调距离。"""

    gain: float = 9.0  # 目标角到指令角的刚度（1/秒），越大越跟手，过大会抖
    deadzone: float = 0.05  # 归一化画面坐标（半幅为 1）内的死区
    smoothing: float = 0.7  # 新一帧观测占的权重，越大越跟手，越小越稳
    max_speed_deg_s: float = 150.0
    # 像素误差换算成角度后打的折扣。视场角是估计值，略小于 1 可避免估大了导致过冲
    aim_scale: float = 0.85
    lead_s: float = 0.08  # 按手的角速度向前预判的时间，用来抵消检测和舵机的延迟
    coast_s: float = 0.45  # 手出画面后沿原方向继续追的时间，便于重新找到
    pan_range_deg: tuple[float, float] = (-70.0, 70.0)  # 相对初始位的偏转范围
    tilt_range_deg: tuple[float, float] = (-35.0, 40.0)
    lost_timeout_s: float = 1.5  # 丢失手这么久后开始回初始位
    min_hand_score: float = 0.5
    # false（默认）：两只臂各跟各的，每只臂跟自己视野里最优的那只手，互不相干。
    # true：两只臂同一时刻至多一只在跟随，由视野更好的那只负责；另一只臂的视野质量要比
    #   当前跟随臂高出 switch_margin 并持续 switch_hold_s 秒才会换臂
    single_tracker: bool = False
    switch_margin: float = 0.25
    switch_hold_s: float = 0.6


@dataclass
class FusedConfig:
    """第 3 步：深度相机全局引导 + 腕部精调。"""

    # false（默认）：指向式引导。深度相机发现手后，把合适的那只臂的腕部相机转过去对准，
    #   手进入腕部视野后交给腕部跟随。只转动，不伸缩。
    # true：伸出式引导。夹爪伸向手并保持 standoff 的距离，腕部相机做小角度精调。
    #   需要更准的外参，且尚未在真机上验证。
    reach: bool = False
    cue_max_age_s: float = 0.3  # 深度相机的观测超过这么久就不再用来引导
    cue_switch_margin_deg: float = 10.0  # 另一只臂要省这么多转角才值得换臂
    # 外参自动标定：腕部相机和深度相机同时看到同一只手时自动积累配对样本
    auto_calibrate: bool = True
    auto_calib_center: float = 0.35  # 只采用手在腕部画面中部（半幅的这个比例以内）的观测
    auto_calib_spacing: float = 0.03  # 相邻样本里手的位置至少相距这么远（米）
    auto_calib_max_samples: int = 600

    standoff: float = 0.25  # 夹爪参考点与手保持的距离（米）
    min_reach: float = 0.22  # 手逼近时夹爪最多缩回到离肩部这么近
    max_reach: float = 0.35  # 夹爪参考点离肩部的最大距离
    linear_speed: float = 0.10  # 引导时末端的最大线速度（米/秒）
    depth_range: tuple[float, float] = (0.20, 1.6)  # 认可的手部深度范围（米），下限接近 D435 的最近测距距离
    switch_hysteresis: float = 0.06  # 换臂的滞回距离（米）
    wrist_trim_limit_deg: float = 25.0  # 腕部精调在引导姿态上叠加的最大角度
    lost_timeout_s: float = 1.5


@dataclass
class DetectorConfig:
    backend: str = "mediapipe"
    model_path: str = "models/hand_landmarker.task"
    max_hands: int = 2
    min_confidence: float = 0.5
    max_fps: float = 60.0  # 每路相机的检测频率上限；高于相机帧率即逐帧检测


@dataclass
class Config:
    mock: bool = False
    # 作为开发试验进程运行：登记进程号；退出时只松开机械臂串口，不收手、不动力矩
    lab: bool = False
    control_hz: float = 30.0
    # 启动并连接设备后自动进入的模式。生产运行设为 follow，进程被自动重启后也能直接恢复
    start_mode: str = "idle"
    server: ServerConfig = field(default_factory=ServerConfig)
    arms: dict[str, ArmConfig] = field(
        default_factory=lambda: {
            # 中柱避让：左臂末端不超过前偏右 30°，右臂不超过前偏左 30°
            "left": ArmConfig(
                calibration_id="left", camera="wrist_left", inward_y_sign=-1.0,
                joint_limits={"shoulder_pan": (-110.0, 30.0)},
            ),
            "right": ArmConfig(
                calibration_id="right", camera="wrist_right", inward_y_sign=1.0,
                joint_limits={"shoulder_pan": (-30.0, 110.0)},
            ),
        }
    )
    head: HeadConfig = field(default_factory=HeadConfig)
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            # 腕部相机（OV9732）用传感器原生的 1280x720
            "wrist_left": CameraConfig(width=1280, height=720, hfov_deg=90.0),
            "wrist_right": CameraConfig(width=1280, height=720, hfov_deg=90.0),
            "head": CameraConfig(type="realsense", hfov_deg=69.0),
        }
    )
    head_camera: str = "head"
    # 初始位（度）。夹爪水平朝前，夹爪参考点约在基座前方 26 cm、高 22 cm 处
    home: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": 0.0,
            "shoulder_lift": -75.0,
            "elbow_flex": 60.0,
            "wrist_flex": 10.0,
            "wrist_roll": -90.0,  # 实测：腕部转到 -90° 时腕部相机画面是正的
            "gripper": 0.0,
        }
    )
    # 休息位（度）。折叠收拢，到位后可以安全释放力矩
    rest: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": 0.0,
            "shoulder_lift": -95.0,
            "elbow_flex": 90.0,
            "wrist_flex": 60.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }
    )
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    keyboard: KeyboardConfig = field(default_factory=KeyboardConfig)
    follow: FollowConfig = field(default_factory=FollowConfig)
    fused: FusedConfig = field(default_factory=FusedConfig)
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    # 标定结果（外参、云台预设）保存位置
    calibration_dir: str = "calibration"

    def path(self, relative: str) -> Path:
        p = Path(relative)
        return p if p.is_absolute() else REPO_ROOT / p


def _merge(obj: Any, data: dict[str, Any], where: str) -> None:
    for key, value in data.items():
        if not hasattr(obj, key):
            raise ValueError(f"配置项 {where}{key} 不存在")
        current = getattr(obj, key)
        if dataclasses.is_dataclass(current) and isinstance(value, dict):
            _merge(current, value, f"{where}{key}.")
        elif isinstance(current, dict) and isinstance(value, dict):
            for name, item in value.items():
                if name in current and dataclasses.is_dataclass(current[name]) and isinstance(item, dict):
                    _merge(current[name], item, f"{where}{key}.{name}.")
                elif current and dataclasses.is_dataclass(next(iter(current.values()))):
                    new = type(next(iter(current.values())))()
                    _merge(new, item, f"{where}{key}.{name}.")
                    current[name] = new
                else:
                    current[name] = item
        elif isinstance(current, tuple):
            setattr(obj, key, tuple(value))
        else:
            setattr(obj, key, value)


def load_config(path: str | os.PathLike | None = None) -> Config:
    cfg = Config()
    p = Path(path) if path else DEFAULT_CONFIG_PATH
    if p.exists():
        data = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
        _merge(cfg, data, "")
    elif path:
        raise FileNotFoundError(f"找不到配置文件 {p}")
    for name, arm in cfg.arms.items():
        if not arm.calibration_id:
            arm.calibration_id = name
    return cfg
