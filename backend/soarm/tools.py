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
def _arm_config(args: argparse.Namespace):
    cfg = load_config(args.config)
    if args.arm not in cfg.arms:
        sys.exit(f"配置里没有名为 {args.arm} 的机械臂")
    if not cfg.arms[args.arm].port:
        sys.exit(f"请先在 config/config.yaml 里填写 arms.{args.arm}.port")
    return cfg


def cmd_calibrate(args: argparse.Namespace) -> int:
    """运行 LeRobot 的标准校准流程，结果写入舵机（需要用手掰动每个关节走完整行程）。

    只有 scan 显示某些关节的限位是出厂值 [0, 4095]，或 watch 核对发现零位明显不对时才需要。
    """
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    cfg = _arm_config(args)
    arm = cfg.arms[args.arm]
    robot = SO101Follower(SO101FollowerConfig(port=arm.port, id=arm.calibration_id, use_degrees=True))
    robot.bus.connect()
    try:
        robot.bus.disable_torque(num_retry=3)
        robot.calibrate()
    finally:
        robot.bus.disconnect(disable_torque=True)
    return 0


def cmd_watch(args: argparse.Namespace) -> int:
    """实时显示各关节角度和行程限位，用手摆姿态来核对校准。只读，不改变力矩状态。"""
    from .kinematics import ARM_JOINTS, ArmKinematics
    from .robot.real import RealArm

    cfg = _arm_config(args)
    arm = RealArm(args.arm, cfg.arms[args.arm], cfg.safety)
    arm.open()
    kin = ArmKinematics()
    print("本臂的行程限位（度）：")
    for joint, (lo, hi) in arm.limits().items():
        print(f"  {joint:14s} {lo:7.1f} .. {hi:7.1f}")
    if arm.found_resting:
        print("\n力矩当前是关闭的，可以用手摆动手臂。把手臂水平向前伸直时，前 5 个关节应都接近 0°。")
    else:
        print("\n注意：力矩当前是开启的，本命令不会改变它。")
    print("Ctrl+C 退出。\n")
    try:
        while True:
            pos = arm.read()
            tip, pitch = kin.tip_and_pitch({j: pos[j] for j in ARM_JOINTS})
            joints = "  ".join(f"{k[:10]}={v:7.1f}" for k, v in pos.items())
            print(f"\r{joints} | 夹爪尖 x={tip[0]:.3f} y={tip[1]:.3f} z={tip[2]:.3f} 俯仰={pitch:5.1f}  ", end="")
            if args.once:
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        arm.disconnect()
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


# ---------------------------------------------------------------- 外参：旁路采集与离线求解
def cmd_collect_calib(args: argparse.Namespace) -> int:
    """不打断正在运行的控制台，旁路采集外参标定样本。

    本进程自己打开深度相机，同时只读轮询运行中控制台的状态接口，拿到机械臂关节角和
    腕部相机看到的手。两边同时看到手时记下一对“视线 + 3D 点”。前提是运行中的控制台
    没有占用深度相机（配置里把它 enabled: false）。
    """
    import json
    import urllib.request
    from functools import partial

    import numpy as np

    from .control.geometry import wrist_ray
    from .kinematics import ARM_JOINTS, ArmKinematics
    from .perception.cameras import CameraWorker
    from .perception.hand import make_detector

    cfg = load_config(args.config)
    head_cfg = cfg.cameras[cfg.head_camera]
    worker = CameraWorker(
        cfg.head_camera, head_cfg, cfg.detector,
        partial(make_detector, cfg.detector.backend, cfg.path(cfg.detector.model_path), 2, cfg.detector.min_confidence),
    )
    worker.start()
    kin = ArmKinematics()
    fc = cfg.fused
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    counts: dict[str, int] = {}
    last_point: dict[str, np.ndarray] = {}
    prev_head: tuple[float, np.ndarray] | None = None
    stats = {"轮询": 0, "深度相机恰有一只手": 0, "有臂在跟随": 0, "两边同时": 0}
    t_end = time.monotonic() + args.seconds
    print(f"开始采集 {args.seconds:.0f} 秒，写入 {out}。Ctrl+C 可提前结束。")
    try:
        with out.open("a", encoding="utf-8") as f:
            while time.monotonic() < t_end:
                time.sleep(0.04)
                if worker.error:
                    print(f"\r深度相机: {worker.error}        ", end="")
                    continue
                stats["轮询"] += 1
                head = worker.primary_hand(max_age_s=0.12)
                # 深度相机画面里必须恰好只有一只手：有多只时无法确定哪只才是腕部相机在跟的那只
                ok_head = (
                    head is not None and head.xyz is not None and worker.hand_count() == 1
                    and fc.depth_range[0] <= head.xyz[2] <= fc.depth_range[1]
                )
                stats["深度相机恰有一只手"] += ok_head
                try:
                    state = json.load(urllib.request.urlopen(f"{args.prod}/api/state", timeout=1))
                except OSError as e:
                    print(f"\r读不到运行中的控制台: {e}        ", end="")
                    continue
                arm = next((n for n, a in state["arms"].items() if a.get("fused_selected")), None)
                stats["有臂在跟随"] += arm is not None
                if ok_head:
                    # 状态接口没有时间戳，两边差几十毫秒；只在手基本不动时采样来消除这个误差
                    slow = prev_head is not None and np.linalg.norm(head.xyz - prev_head[1]) < 0.012 * max(1.0, (head.t - prev_head[0]) / 0.033)
                    prev_head = (head.t, np.array(head.xyz))
                else:
                    slow, prev_head = False, None
                if not (ok_head and arm and slow):
                    continue
                a = state["arms"][arm]
                cam_state = state["cameras"].get(cfg.arms[arm].camera if arm in cfg.arms else "", {})
                wrist = cam_state.get("primary")
                if not wrist or a["status"] not in ("跟随中", "已对准"):
                    continue
                stats["两边同时"] += 1
                if max(abs(wrist["cx"] - 0.5), abs(wrist["cy"] - 0.5)) * 2.0 > fc.auto_calib_center:
                    continue
                if arm in last_point and np.linalg.norm(last_point[arm] - head.xyz) < fc.auto_calib_spacing:
                    continue
                width, height = cam_state.get("size", [1280, 720])
                origin, direction = wrist_ray(
                    kin, {j: a["joints"][j] for j in ARM_JOINTS}, wrist["cx"], wrist["cy"], args.hfov, height / width
                )
                last_point[arm] = np.array(head.xyz)
                counts[arm] = counts.get(arm, 0) + 1
                f.write(json.dumps({
                    "arm": arm, "origin": origin.tolist(), "direction": direction.tolist(),
                    "cam_point": [float(v) for v in head.xyz], "wrist": [wrist["cx"], wrist["cy"]],
                    "joints": a["joints"], "time": time.time(),
                }) + "\n")
                f.flush()
                print(f"\r已采集 {counts}        ", end="")
    except KeyboardInterrupt:
        pass
    finally:
        worker.stop()
    print(f"\n结束。{stats}，新增样本 {counts}")
    return 0


def cmd_solve_calib(args: argparse.Namespace) -> int:
    """用旁路采集的样本离线求解外参，写入 calibration/extrinsics.yaml。"""
    import json

    import numpy as np

    from .calib import ArmExtrinsic, BearingSample, Calibration, solve_from_bearings

    cfg = load_config(args.config)
    rows = [json.loads(line) for line in Path(args.samples).read_text(encoding="utf-8").splitlines() if line.strip()]
    path = cfg.path(cfg.calibration_dir) / "extrinsics.yaml"
    calibration = Calibration.load(path)
    solved = 0
    for arm in sorted({r["arm"] for r in rows}):
        samples = [
            BearingSample(np.array(r["origin"]), np.array(r["direction"]), np.array(r["cam_point"]))
            for r in rows if r["arm"] == arm
        ]
        pts = np.array([s.cam_point for s in samples])
        print(f"{arm}: {len(samples)} 个样本，手在相机系下的范围 "
              f"x {pts[:, 0].min():.2f}..{pts[:, 0].max():.2f}  y {pts[:, 1].min():.2f}..{pts[:, 1].max():.2f}  "
              f"深度 {pts[:, 2].min():.2f}..{pts[:, 2].max():.2f} m")
        try:
            T, rms, used = solve_from_bearings(samples)
        except ValueError as e:
            print(f"  无法求解：{e}")
            continue
        print(f"  采用 {used} 个样本，手偏离视线的 RMS {rms * 1000:.0f} mm")
        print(f"  相机在该臂基座系下的位置 x={T[0, 3]:.3f} y={T[1, 3]:.3f} z={T[2, 3]:.3f} m；"
              f"光轴方向 {np.round(T[:3, 2], 2).tolist()}")
        calibration.arms[arm] = ArmExtrinsic(T, rms, used)
        solved += 1
    if solved and not args.dry_run:
        calibration.head_preset = calibration.head_preset or {"pan": 0.0, "tilt": 0.0}
        calibration.save(path)
        print(f"已保存到 {path}")
    return 0 if solved else 1


# 手眼标定时让手臂往前上方伸到深度相机下面的姿态：(肩部俯仰, 肘部, 腕部俯仰)
_BODY_SHAPES = [(-20, 10, 0), (-10, 0, -10), (-25, 25, -15), (0, -10, -10), (-30, 15, 5), (-40, 20, -5)]
# 肩部旋转相对“正前方”的偏角，负值表示朝内侧（朝中柱）。实测相机视野只覆盖两臂中间，
# 手臂要朝内转 12° 以上才进画面；上限受 30° 的中柱避让限位约束
_BODY_PANS_OUTWARD = [-28, -23, -18, -13]


def cmd_body_calib(args: argparse.Namespace) -> int:
    """用机械臂自身做手眼标定：让一只臂摆若干姿态，用深度相机看到的手臂点云对运动学骨架。

    不需要人手和标定板。要求这只臂的串口当前没有被别的程序占用（生产运行时先
    scripts/lab.sh lend <臂> 借出来）。被要求让路时会立刻松开串口并退出，舵机力矩不变。
    """
    import json

    import cv2
    import numpy as np

    from . import lab
    from .calib import ArmExtrinsic, BodyObservation, Calibration, fit_floor_normal, solve_from_body, verify_body
    from .control.controller import Controller
    from .kinematics import ARM_JOINTS
    from .perception.cameras import CameraWorker, deproject
    from .robot.real import RealArm

    cfg = load_config(args.config)
    name = args.arm
    if name not in cfg.arms or not cfg.arms[name].port:
        sys.exit(f"请先在配置里填写 arms.{name}.port")
    cfg.arms = {name: cfg.arms[name]}
    head_cfg = cfg.cameras[cfg.head_camera]
    head_cfg.detect = False
    out_dir = cfg.path(cfg.calibration_dir) / f"body_{name}"
    out_dir.mkdir(parents=True, exist_ok=True)

    lab.claim()
    worker = CameraWorker(cfg.head_camera, head_cfg, cfg.detector, lambda: None)
    worker.start()
    ctl = Controller(cfg, {name: RealArm(name, cfg.arms[name], cfg.safety)}, None, {cfg.head_camera: worker})
    finished = False
    try:
        ctl.start()
        rt = ctl.arms[name]
        if not rt.arm.connected:
            sys.exit(f"无法连接 {name}: {rt.arm.error}")
        for _ in range(60):
            if worker.latest() is not None and worker.intrinsics is not None:
                break
            time.sleep(0.1)
        else:
            sys.exit(f"深度相机没有画面: {worker.error}")
        intr = worker.intrinsics

        def depth_median(n: int = 8) -> tuple[np.ndarray, np.ndarray]:
            frames, color, last = [], None, 0.0
            while len(frames) < n:
                color, depth, t = worker.latest()
                if t > last:
                    frames.append(depth)
                    last = t
                time.sleep(0.02)
            stack = np.stack(frames)
            stack[stack <= 0.05] = np.nan
            with np.errstate(all="ignore"):
                med = np.nanmedian(stack, axis=0)
            return color, np.nan_to_num(med, nan=0.0)

        # 背景：手臂还在连接时的姿态（不在相机视野里）
        _, background = depth_median(12)
        up_cam, cam_height = fit_floor_normal(background, (intr.fx, intr.fy, intr.cx, intr.cy))
        pitch_down = float(np.degrees(np.arcsin(-up_cam[2])))
        print(f"由地面测得：相机低头 {pitch_down:.1f}°，离地 {cam_height:.2f} m")
        outward = 1.0 if rt.cfg.inward_y_sign > 0 else -1.0  # 右臂朝外是 +pan（向右），左臂相反
        poses = []
        for k in range(len(_BODY_SHAPES)):  # 先让每个转角都出现一遍，再换形状，这样提前中断时数据也均匀
            for i, pan in enumerate(_BODY_PANS_OUTWARD):
                lift, elbow, wf = _BODY_SHAPES[(i + k) % len(_BODY_SHAPES)]
                poses.append({"shoulder_pan": outward * pan, "shoulder_lift": lift, "elbow_flex": elbow,
                              "wrist_flex": wf, "wrist_roll": rt.home["wrist_roll"], "gripper": 0.0})
        poses = poses[: args.poses]
        observations, records, depths = [], [], []
        print(f"{name} 臂将依次摆 {len(poses)} 个姿态，每个约 3-5 秒。Ctrl+C 或被要求让路时立即停止。")
        for i, pose in enumerate(poses):
            ctl.goto_pose(name, pose, "calib")
            deadline = time.monotonic() + 20
            while ctl.is_moving(name) and time.monotonic() < deadline:
                time.sleep(0.05)
            if ctl.is_moving(name) or "阻挡" in rt.status:
                print(f"  姿态 {i + 1}: 没能到位（{rt.status}），跳过")
                continue
            time.sleep(0.9)
            color, depth = depth_median()
            joints = {j: rt.pos[j] for j in ARM_JOINTS}
            # 前景：比背景近的点。只取 0.7 米以内：手臂离相机 0.2-0.6 米，再远的是路过的人
            fg = (depth > 0.12) & (depth < 0.7) & ((background < 0.05) | (depth < background - 0.04))
            fg = cv2.erode(fg.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
            vs, us = np.nonzero(fg)
            if len(vs) > 600:
                pick = np.random.default_rng(i).choice(len(vs), 600, replace=False)
                vs, us = vs[pick], us[pick]
            pts = np.array([deproject(intr, u, v, depth[v, u]) for u, v in zip(us, vs)]).reshape(-1, 3)
            skeleton = ctl.kin.skeleton(joints)
            print(f"  姿态 {i + 1}/{len(poses)}: pan {joints['shoulder_pan']:.0f}° "
                  f"夹爪尖 ({skeleton[-1][0]:.2f}, {skeleton[-1][1]:.2f}, {skeleton[-1][2]:.2f})  前景点 {len(pts)}")
            cv2.imwrite(str(out_dir / f"pose_{i:02d}.jpg"), color)
            np.save(out_dir / f"pose_{i:02d}_depth.npy", depth.astype(np.float32))
            records.append({"joints": joints, "skeleton": skeleton.tolist(), "points": pts.tolist()})
            depths.append(depth)
            if len(pts) >= 40:
                observations.append(BodyObservation(skeleton, pts))
        (out_dir / "observations.json").write_text(json.dumps({
            "up_cam": up_cam.tolist(), "intrinsics": [intr.fx, intr.fy, intr.cx, intr.cy], "records": records,
        }))
        np.save(out_dir / "background_depth.npy", background.astype(np.float32))
        print(f"有效姿态 {len(observations)}/{len(poses)}，原始数据已存到 {out_dir}")
        yaw = None if args.yaw != args.yaw else args.yaw  # nan 表示不固定
        # 残差最小的解不一定对（可能沿手臂错开一节连杆），所以拿所有候选解去做反向验证，取得分最高的
        skeleton_list = [np.array(r["skeleton"]) for r in records]
        intr4 = (intr.fx, intr.fy, intr.cx, intr.cy)
        scored = []
        for cand_T, cand_rms, cand_used in solve_from_body(observations, up_cam, yaw_deg=yaw, all_candidates=True):
            c_inside, c_confirmed = verify_body(cand_T, skeleton_list, depths, intr4)
            print(f"  候选解 位置 ({cand_T[0, 3]:.3f}, {cand_T[1, 3]:.3f}, {cand_T[2, 3]:.3f}) RMS {cand_rms * 1000:.0f} mm"
                  f"  反向验证 画面内 {c_inside:.0%} 被证实 {c_confirmed:.0%}")
            scored.append((c_inside * c_confirmed, cand_T, cand_rms, cand_used))
        _, T, rms, used = max(scored, key=lambda item: item[0])
        print(f"求解：采用 {used} 个点，点到手臂表面的 RMS {rms * 1000:.0f} mm")
        print(f"  相机在 {name} 臂基座系下的位置 x={T[0, 3]:.3f} y={T[1, 3]:.3f} z={T[2, 3]:.3f} m；"
              f"光轴方向 {np.round(T[:3, 2], 2).tolist()}")
        # 把骨架投影回每张彩色图，供人工核对
        T_inv = np.linalg.inv(T)
        for i, rec in enumerate(records):
            img = cv2.imread(str(out_dir / f"pose_{i:02d}.jpg"))
            if img is None:
                continue
            cam = np.array(rec["skeleton"]) @ T_inv[:3, :3].T + T_inv[:3, 3]
            px = [(int(intr.fx * p[0] / p[2] + intr.cx), int(intr.fy * p[1] / p[2] + intr.cy)) for p in cam if p[2] > 0.05]
            for a, b in zip(px[:-1], px[1:]):
                cv2.line(img, a, b, (0, 255, 255), 2)
            for p in px:
                cv2.circle(img, p, 4, (0, 0, 255), -1)
            cv2.imwrite(str(out_dir / f"check_{i:02d}.jpg"), img)
        inside, confirmed = verify_body(
            T, [np.array(r["skeleton"]) for r in records], depths, (intr.fx, intr.fy, intr.cx, intr.cy)
        )
        print(f"反向验证：手臂模型有 {inside:.0%} 落在画面内，其中 {confirmed:.0%} 被深度图证实")
        trusted = inside >= 0.25 and confirmed >= 0.8
        if not trusted:
            print("验证未通过：这个外参不可信，不保存。请看 check_*.jpg 里黄色骨架是否压在手臂上。")
        if trusted and not args.dry_run:
            path = cfg.path(cfg.calibration_dir) / "extrinsics.yaml"
            calibration = Calibration.load(path)
            calibration.arms[name] = ArmExtrinsic(T, rms, used, up_cam)
            calibration.save(path)
            print(f"已保存到 {path}")
        finished = True
        return 0 if trusted else 1
    except ValueError as e:
        print(f"无法求解：{e}")
        finished = True
        return 1
    except KeyboardInterrupt:
        print("\n被要求让路：立即松开串口，舵机力矩保持。")
        return 130
    finally:
        if finished:
            ctl.shutdown()  # 正常结束：收到休息位并释放力矩
        else:
            ctl.abort()  # 中途让路：不收手、不动力矩，生产会接管
        worker.stop()
        lab.release()


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

    p = sub.add_parser("calibrate", help="运行 LeRobot 标准校准流程（一般不需要）")
    p.add_argument("arm")
    p.set_defaults(func=cmd_calibrate)

    p = sub.add_parser("watch", help="实时显示关节角度与行程限位（只读）")
    p.add_argument("arm")
    p.add_argument("--once", action="store_true", help="只读一次就退出")
    p.set_defaults(func=cmd_watch)

    p = sub.add_parser("collect-calib", help="旁路采集外参标定样本，不打断运行中的控制台")
    p.add_argument("--prod", default="http://127.0.0.1:8000", help="运行中控制台的地址")
    p.add_argument("--out", default="calibration/bearing_samples.jsonl")
    p.add_argument("--seconds", type=float, default=600)
    p.add_argument("--hfov", type=float, default=90.0, help="运行中控制台的腕部相机水平视场角")
    p.set_defaults(func=cmd_collect_calib)

    p = sub.add_parser("solve-calib", help="用旁路采集的样本离线求解外参")
    p.add_argument("samples", nargs="?", default="calibration/bearing_samples.jsonl")
    p.add_argument("--dry-run", action="store_true", help="只求解和打印，不保存")
    p.set_defaults(func=cmd_solve_calib)

    p = sub.add_parser("body-calib", help="用机械臂自身做手眼标定（不需要人手）")
    p.add_argument("arm")
    p.add_argument("--poses", type=int, default=16)
    p.add_argument("--yaw", type=float, default=0.0,
                   help="相机朝向相对机械臂朝向的偏角（度），默认 0 即朝向一致；给 nan 则连它一起解")
    p.add_argument("--dry-run", action="store_true", help="只采集和求解，不保存外参")
    p.set_defaults(func=cmd_body_calib)

    p = sub.add_parser("head-watch", help="释放云台力矩并显示原始刻度")
    p.set_defaults(func=cmd_head_watch)

    args = parser.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
