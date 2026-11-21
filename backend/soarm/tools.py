"""上电后的一次性准备工具。用法：soarm-tools <子命令> --help"""

from __future__ import annotations

import argparse
import glob
import os
import sys
import time
from pathlib import Path

from .config import load_config

BAUDRATES = (1_000_000, 500_000, 115_200, 57_600)


def _serial_ports() -> list[str]:
    stable = sorted(glob.glob("/dev/serial/by-id/*"))
    if stable:
        return stable
    return sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))


# ---------------------------------------------------------------- scan
def cmd_scan(args: argparse.Namespace) -> int:
    """扫描所有串口上的 STS3215 舵机，用来分辨哪块板是左臂/右臂/云台。"""
    import rustypot

    _, definition = rustypot.find_model("sts3215")
    ports = [args.port] if args.port else _serial_ports()
    if not ports:
        print("没有发现串口。请确认控制板的 USB 已接入，且跳线帽在 B 位（USB ↔ SERVO）。")
        return 1
    for port in ports:
        print(f"\n串口 {port}  ->  {os.path.realpath(port)}")
        found: dict[int, int] = {}
        baud_ok = None
        for baud in BAUDRATES:
            try:
                bus = rustypot.Bus(port, baud, 0.05, {})
            except OSError as e:
                print(f"  打不开：{e}（若为权限问题：sudo usermod -aG dialout $USER 后重新登录）")
                break
            try:
                found = bus.broadcast_scan(definition)
            except Exception as e:
                print(f"  {baud} 波特率扫描出错：{e}")
                found = {}
            finally:
                bus.close()
            if found:
                baud_ok = baud
                break
        if not found:
            print("  没有舵机应答。舵机需要外部电源（DC 口）供电，仅插 USB 不会应答。")
            continue
        ids = sorted(found)
        print(f"  波特率 {baud_ok}，发现 {len(ids)} 个舵机，ID = {ids}")
        bus = rustypot.Bus(port, baud_ok, 0.05, {i: definition for i in ids})
        try:
            for i in ids:
                pos = bus.read_register(i, "present_position", 2)
                volt = bus.read_register(i, "present_voltage", 2)
                temp = bus.read_register(i, "present_temperature", 2)
                lo = bus.read_register(i, "min_position_limit", 2)
                hi = bus.read_register(i, "max_position_limit", 2)
                off = bus.read_register(i, "homing_offset", 2)
                print(f"    ID {i}: 位置 {pos}  电压 {volt}  温度 {temp}  限位 [{lo}, {hi}]  偏置 {off}")
        finally:
            bus.close()
        if ids == [1, 2, 3, 4, 5, 6]:
            print("  判断：这是一只机械臂（ID 1-6）。晃动手臂或看接线分辨左右，填入 arms.left.port / arms.right.port")
        elif len(ids) == 2:
            print(f"  判断：这是云台。填入 head.port，并设置 head.pan.id / head.tilt.id 为 {ids}")
        else:
            print("  判断：舵机数量不是 6 也不是 2，请对照硬件确认。")
    return 0


# ---------------------------------------------------------------- cameras
def _udev_props(device: str) -> dict[str, str]:
    import subprocess

    try:
        out = subprocess.run(
            ["udevadm", "info", "-q", "property", "-n", device], capture_output=True, text=True, timeout=5
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return {}
    return dict(line.split("=", 1) for line in out.splitlines() if "=" in line)


def cmd_cameras(args: argparse.Namespace) -> int:
    """列出相机并各存一张快照，用来分辨哪个是左腕/右腕相机。"""
    import cv2

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    # 两个腕部相机同型号且序列号相同，by-id 会重名，只能按 USB 口区分。
    # 必须用带 usbv2 / usbv3 的路径：不带版本号的 by-path 在 USB2 与 USB3 的同号口上会重名。
    print("UVC 相机（路径按 USB 口区分：相机换插到别的口后需要重新填写）：")
    for path in sorted(glob.glob("/dev/v4l/by-path/*-usbv*-video-index0")):
        props = _udev_props(path)
        model = props.get("ID_MODEL", "未知型号")
        if "RealSense" in model:
            continue  # RealSense 走 SDK，按序列号打开
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        ok, frame = cap.read() if cap.isOpened() else (False, None)
        cap.release()
        if ok:
            snap = out / (Path(path).name + ".jpg")
            cv2.imwrite(str(snap), frame)
            print(f"  {path}\n      {model}  {frame.shape[1]}x{frame.shape[0]}  快照: {snap}")
        else:
            print(f"  {path}\n      {model}  无法取帧（可能正被占用）")
    try:
        import pyrealsense2 as rs

        devices = list(rs.context().query_devices())
        print("RealSense：" + ("" if devices else "未发现"))
        for dev in devices:
            print(
                f"  {dev.get_info(rs.camera_info.name)}  序列号 {dev.get_info(rs.camera_info.serial_number)}"
                "（配置里填这个；它与 USB 描述符里的芯片序列号不同）"
                f"  固件 {dev.get_info(rs.camera_info.firmware_version)}"
                f"  USB {dev.get_info(rs.camera_info.usb_type_descriptor)}"
            )
    except Exception as e:
        print(f"RealSense 查询失败：{e}")
    return 0


# ---------------------------------------------------------------- 校准
def _follower(args: argparse.Namespace):
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    cfg = load_config(args.config)
    if args.arm not in cfg.arms:
        sys.exit(f"配置里没有名为 {args.arm} 的机械臂")
    arm = cfg.arms[args.arm]
    if not arm.port:
        sys.exit(f"请先在 config/config.yaml 里填写 arms.{args.arm}.port")
    return SO101Follower(SO101FollowerConfig(port=arm.port, id=arm.calibration_id, use_degrees=True))


def cmd_recover_calibration(args: argparse.Namespace) -> int:
    """从舵机里读回上一次校准（零点偏置和限位），存成 LeRobot 校准文件，免去重新校准。"""
    robot = _follower(args)
    bus = robot.bus
    bus.connect()
    try:
        calib = bus.read_calibration()
    finally:
        bus.disconnect(disable_torque=False)
    print(f"{args.arm} 臂舵机内保存的校准：")
    suspicious = []
    for name, c in calib.items():
        span = c.range_max - c.range_min
        note = ""
        if span <= 0 or span >= 4090:
            note = "  <- 限位是出厂默认值，说明这个关节没有被校准过"
            suspicious.append(name)
        print(f"  {name:14s} 偏置 {c.homing_offset:6d}  范围 [{c.range_min}, {c.range_max}]{note}")
    # wrist_roll 在 LeRobot 校准里本来就是整圈，不算异常
    suspicious = [n for n in suspicious if n != "wrist_roll"]
    if suspicious and not args.force:
        print(f"\n以上关节 {suspicious} 没有有效校准，未保存。请改用：soarm-tools calibrate {args.arm}")
        return 1
    robot.calibration = calib
    robot._save_calibration()
    print(f"\n已保存到 {robot.calibration_fpath}")
    print(f"请再运行 soarm-tools watch {args.arm} 核对角度是否合理。")
    return 0


def cmd_calibrate(args: argparse.Namespace) -> int:
    """运行 LeRobot 的标准校准流程（需要用手掰动每个关节走完整行程）。"""
    robot = _follower(args)
    robot.bus.connect()
    try:
        robot.bus.disable_torque(num_retry=3)
        robot.calibrate()
    finally:
        robot.bus.disconnect(disable_torque=True)
    return 0


def cmd_watch(args: argparse.Namespace) -> int:
    """释放力矩并实时显示各关节角度，用手摆姿态来核对校准。"""
    from .kinematics import ARM_JOINTS, ArmKinematics

    robot = _follower(args)
    if not robot.calibration:
        sys.exit(f"没有校准文件 {robot.calibration_fpath}")
    kin = ArmKinematics()
    bus = robot.bus
    bus.connect()
    print("力矩已释放，请用手托住手臂。把手臂水平向前伸直时，前 5 个关节应都接近 0°。Ctrl+C 退出。\n")
    try:
        bus.disable_torque(num_retry=3)
        while True:
            pos = bus.sync_read("Present_Position", num_retry=2)
            tip, pitch = kin.tip_and_pitch({j: pos[j] for j in ARM_JOINTS})
            joints = "  ".join(f"{k[:10]}={v:7.1f}" for k, v in pos.items())
            print(f"\r{joints} | 夹爪尖 x={tip[0]:.3f} y={tip[1]:.3f} z={tip[2]:.3f} 俯仰={pitch:5.1f}  ", end="")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
    finally:
        bus.disconnect(disable_torque=True)
    return 0


def cmd_head_watch(args: argparse.Namespace) -> int:
    """释放云台力矩并显示原始刻度，用来确定 head.*.center / min / max。"""
    import rustypot

    cfg = load_config(args.config)
    if not cfg.head.port:
        sys.exit("请先在 config/config.yaml 里填写 head.port")
    _, definition = rustypot.find_model("sts3215")
    ids = {"pan": cfg.head.pan.id, "tilt": cfg.head.tilt.id}
    bus = rustypot.Bus(cfg.head.port, 1_000_000, 0.05, {i: definition for i in ids.values()})
    print("云台力矩已释放。把相机摆到“朝前”的位置，记下刻度作为 center；")
    print("再分别摆到四个方向的极限，记下刻度作为 min / max。Ctrl+C 退出。\n")
    try:
        for i in ids.values():
            bus.write_register(i, "torque_enable", 0)
        while True:
            text = "  ".join(f"{axis}(ID {i}) = {bus.read_register(i, 'present_position', 2)}" for axis, i in ids.items())
            print(f"\r{text}    ", end="")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
    finally:
        bus.close()
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(prog="soarm-tools", description="SO-ARM 演示套件的上电准备工具")
    parser.add_argument("--config", help="配置文件路径，默认 config/config.yaml")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("scan", help="扫描串口上的舵机")
    p.add_argument("--port", help="只扫描这个串口")
    p.set_defaults(func=cmd_scan)

    p = sub.add_parser("cameras", help="列出相机并保存快照")
    p.add_argument("--out", default="/tmp/soarm-cameras", help="快照保存目录")
    p.set_defaults(func=cmd_cameras)

    p = sub.add_parser("recover-calibration", help="从舵机读回已有校准并保存")
    p.add_argument("arm")
    p.add_argument("--force", action="store_true", help="即使有关节看起来未校准也保存")
    p.set_defaults(func=cmd_recover_calibration)

    p = sub.add_parser("calibrate", help="运行 LeRobot 标准校准流程")
    p.add_argument("arm")
    p.set_defaults(func=cmd_calibrate)

    p = sub.add_parser("watch", help="释放力矩并实时显示关节角度")
    p.add_argument("arm")
    p.set_defaults(func=cmd_watch)

    p = sub.add_parser("head-watch", help="释放云台力矩并显示原始刻度")
    p.set_defaults(func=cmd_head_watch)

    args = parser.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
