"""相机采集线程：取帧、跑手部检测、锁定“主手”、输出带标注的 JPEG。

每路相机一个 CameraWorker。相机打不开不会让程序退出，而是标记错误并定时重试，
这样没插相机或没上电时控制台照样能起来。
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

from ..config import CameraConfig, DetectorConfig
from .hand import HAND_CONNECTIONS, Hand, HandDetector, view_quality

logger = logging.getLogger(__name__)

_ROTATE = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
_RETRY_S = 3.0


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


class _OpenCVSource:
    def __init__(self, cfg: CameraConfig):
        device: int | str = int(cfg.device) if str(cfg.device).isdigit() else cfg.device
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开相机 {cfg.device!r}")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
        self.cap.set(cv2.CAP_PROP_FPS, cfg.fps)
        # 缓冲不能只留 1 帧：实测两个腕部相机同时采集时，1 帧缓冲会让驱动没有空缓冲可写而丢帧，
        # 帧率掉到 20 帧左右；4 帧缓冲下两路都是稳定的 30 帧。采集线程持续取帧，不会积压出延迟
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)
        self.intrinsics: Intrinsics | None = None

    def read(self) -> tuple[np.ndarray, np.ndarray | None]:
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("相机读帧失败")
        return frame, None

    def close(self) -> None:
        self.cap.release()


class _RealSenseSource:
    """彩色 + 对齐到彩色的深度（米）。"""

    def __init__(self, cfg: CameraConfig):
        import pyrealsense2 as rs

        self._rs = rs
        self.pipeline = rs.pipeline()
        rs_cfg = rs.config()
        if cfg.device:
            rs_cfg.enable_device(str(cfg.device))
        rs_cfg.enable_stream(rs.stream.color, cfg.width, cfg.height, rs.format.bgr8, cfg.fps)
        rs_cfg.enable_stream(rs.stream.depth, cfg.width, cfg.height, rs.format.z16, cfg.fps)
        profile = self.pipeline.start(rs_cfg)
        self._scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self._align = rs.align(rs.stream.color)
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intrinsics: Intrinsics | None = Intrinsics(intr.fx, intr.fy, intr.ppx, intr.ppy)

    def read(self) -> tuple[np.ndarray, np.ndarray | None]:
        frames = self._align.process(self.pipeline.wait_for_frames(2000))
        color, depth = frames.get_color_frame(), frames.get_depth_frame()
        if not color or not depth:
            raise RuntimeError("RealSense 丢帧")
        return (
            np.asanyarray(color.get_data()).copy(),
            np.asanyarray(depth.get_data()).astype(np.float32) * self._scale,
        )

    def close(self) -> None:
        self.pipeline.stop()


def deproject(intr: Intrinsics, u: float, v: float, depth_m: float) -> np.ndarray:
    """像素 + 深度 -> 相机光学系坐标（x 右、y 下、z 前，米）。"""
    return np.array([(u - intr.cx) / intr.fx * depth_m, (v - intr.cy) / intr.fy * depth_m, depth_m])


def depth_at(depth: np.ndarray, u: float, v: float, radius: int = 6) -> float | None:
    """取 (u, v) 周围小块的有效深度中位数；无有效值返回 None。"""
    h, w = depth.shape
    u0, v0 = int(round(u)), int(round(v))
    patch = depth[max(0, v0 - radius) : min(h, v0 + radius + 1), max(0, u0 - radius) : min(w, u0 + radius + 1)]
    valid = patch[patch > 0.05]
    if valid.size < 5:
        return None
    return float(np.median(valid))


class _Grabber:
    """持续取帧的线程，只保留最新一帧。"""

    def __init__(self, source, rotate: int):
        self._source = source
        self._rotate = rotate
        self._cond = threading.Condition()
        self._latest: tuple[int, np.ndarray, np.ndarray | None, float] | None = None
        self._stopped = False
        self.error: str | None = None
        self.fps = 0.0
        self.read_ms = 0.0
        self._thread = threading.Thread(target=self._run, name="grab", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stopped = True
        self._thread.join(timeout=3)

    def _run(self) -> None:
        seq, prev = 0, time.monotonic()
        while not self._stopped:
            try:
                t0 = time.monotonic()
                frame, depth = self._source.read()
            except Exception as e:
                self.error = str(e)
                with self._cond:
                    self._cond.notify_all()
                return
            now = time.monotonic()
            self.read_ms = 0.9 * self.read_ms + 0.1 * (now - t0) * 1e3
            self.fps = 0.9 * self.fps + 0.1 / max(now - prev, 1e-3)
            prev = now
            if self._rotate in _ROTATE:
                frame = cv2.rotate(frame, _ROTATE[self._rotate])
                if depth is not None:
                    depth = cv2.rotate(depth, _ROTATE[self._rotate])
            seq += 1
            with self._cond:
                self._latest = (seq, frame, depth, now)
                self._cond.notify_all()

    def wait_newer(self, seq: int, timeout: float):
        """等到比 seq 更新的一帧，返回 (序号, 画面, 深度, 采集时刻)；超时或出错返回 None。"""
        with self._cond:
            self._cond.wait_for(lambda: self.error or (self._latest and self._latest[0] > seq), timeout)
            if self._latest is None or self._latest[0] <= seq:
                return None
            return self._latest


class RemoteHandCamera:
    """只读轮询另一个正在运行的控制台，把它某路相机的“主手”当作本地相机的检测结果。

    用于开发试验：腕部相机被生产进程占着时，借来的机械臂仍然能知道自己的腕部相机看到了什么。
    cfg.device 的格式是 "http://主机:端口#相机名"。只有检测结果，没有画面。
    """

    def __init__(self, name: str, cfg: CameraConfig):
        self.name, self.cfg = name, cfg
        self._url, _, self._remote = cfg.device.partition("#")
        self._remote = self._remote or name
        self._hand: Hand | None = None
        self._hand_time = 0.0
        self._count = 0
        self.intrinsics = None
        self.error: str | None = "未启动"
        self.detect_error: str | None = None
        self.fps = self.detect_fps = self.read_ms = self.detect_ms = 0.0
        self.detect_enabled = True
        self.overlay_text, self.overlay_active, self.deadzone = "", False, 0.0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"remote-{name}", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)

    def _run(self) -> None:
        import json
        import urllib.request

        prev = time.monotonic()
        while not self._stop.is_set():
            try:
                state = json.load(urllib.request.urlopen(f"{self._url}/api/state", timeout=1))
                cam = state["cameras"][self._remote]
                self.cfg.width, self.cfg.height = cam.get("size", [self.cfg.width, self.cfg.height])
                p = cam.get("primary")
                now = time.monotonic()
                self.fps = self.detect_fps = 0.9 * self.fps + 0.1 / max(now - prev, 1e-3)
                prev = now
                self._count = cam.get("hands", 0)
                self.error = cam.get("error")
                if p:
                    # 对方的状态没有时间戳；按一次检测加一次轮询的典型延迟往前推
                    self._hand = Hand(p["cx"], p["cy"], p["size"], p["score"], np.zeros((21, 2), np.float32), None, now - 0.05)
                    self._hand_time = now
            except Exception as e:
                self.error = f"读不到 {self._url}: {e}"
            self._stop.wait(0.03)

    def primary_hand(self, max_age_s: float = 0.4) -> Hand | None:
        if self._hand is None or time.monotonic() - self._hand_time > max_age_s:
            return None
        return self._hand

    def hand_count(self) -> int:
        return self._count

    def latest(self):
        return None

    def point_at(self, u_norm: float, v_norm: float):
        return None

    def jpeg(self, quality: int = 70) -> bytes:
        frame = np.full((360, 640, 3), 28, np.uint8)
        cv2.putText(frame, "REMOTE (no image)", (150, 190), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (90, 90, 110), 2)
        return cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])[1].tobytes()


class CameraWorker:
    def __init__(self, name: str, cfg: CameraConfig, det_cfg: DetectorConfig, detector_factory):
        self.name = name
        self.cfg = cfg
        self._det_cfg = det_cfg
        self._detector_factory = detector_factory
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._depth: np.ndarray | None = None
        self._hands: list[Hand] = []
        self._primary: Hand | None = None
        self._primary_time = 0.0
        self._frame_time = 0.0
        self.intrinsics: Intrinsics | None = None
        self.error: str | None = "未启动" if cfg.enabled else "已在配置中停用"
        self.fps = 0.0
        self.detect_fps = 0.0
        self.read_ms = 0.0  # 取一帧的平均耗时（含等待下一帧）
        self.detect_ms = 0.0  # 单帧检测的平均耗时
        # 由控制层设置：当前模式用不到这路相机的检测时关掉，省出 CPU
        self.detect_enabled = True
        # 由控制层设置，仅用于画面标注
        self.overlay_text = ""
        self.overlay_active = False
        self.deadzone = 0.0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"cam-{name}", daemon=True)

    # ---------- 生命周期 ----------
    def start(self) -> None:
        if self.cfg.enabled:
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=3)

    def _open(self):
        if self.cfg.type == "realsense":
            return _RealSenseSource(self.cfg)
        if self.cfg.type == "opencv":
            if self.cfg.device == "":
                raise RuntimeError("未配置 device")
            return _OpenCVSource(self.cfg)
        raise RuntimeError(f"未知相机类型 {self.cfg.type}")

    def _run(self) -> None:
        detector: HandDetector | None = None
        if self.cfg.detect:
            try:
                detector = self._detector_factory()
            except Exception as e:  # 模型缺失等：相机仍可看画面，只是没有检测
                logger.error("相机 %s 的手部检测器初始化失败: %s", self.name, e)
                self.detect_error = str(e)
        min_interval = 1.0 / self._det_cfg.max_fps
        while not self._stop.is_set():
            try:
                source = self._open()
            except Exception as e:
                self.error = str(e)
                self._stop.wait(_RETRY_S)
                continue
            self.intrinsics = source.intrinsics
            self.error = None
            logger.info("相机 %s 已打开", self.name)
            grabber = _Grabber(source, self.cfg.rotate)
            grabber.start()
            last_detect = 0.0
            seq = 0
            try:
                while not self._stop.is_set():
                    # 采集在独立线程里持续进行，这里总是拿最新的一帧来检测，
                    # 检测慢的时候跳过旧帧而不是排队，延迟不会累积
                    got = grabber.wait_newer(seq, timeout=1.0)
                    if got is None:
                        if grabber.error:
                            raise RuntimeError(grabber.error)
                        continue
                    seq, frame, depth, now = got
                    self.fps, self.read_ms = grabber.fps, grabber.read_ms
                    hands = None
                    if detector is not None and not self.detect_enabled:
                        hands, self.detect_fps = [], 0.0
                    elif detector is not None and now - last_detect >= min_interval:
                        self.detect_fps = 0.8 * self.detect_fps + 0.2 / max(now - last_detect, 1e-3)
                        last_detect = now
                        t_detect = time.monotonic()
                        hands = detector.detect(frame, int(now * 1000))
                        self.detect_ms = 0.9 * self.detect_ms + 0.1 * (time.monotonic() - t_detect) * 1e3
                        for hand in hands:
                            hand.t = now  # 记这一帧的采集时刻，而不是检测完成的时刻
                        if depth is not None and self.intrinsics is not None:
                            self._attach_depth(hands, depth, frame.shape)
                    with self._lock:
                        self._frame, self._depth, self._frame_time = frame, depth, now
                        if hands is not None:
                            self._hands = hands
                            self._update_primary(hands, now)
            except Exception as e:
                self.error = str(e)
                logger.warning("相机 %s 出错: %s", self.name, e)
            finally:
                grabber.stop()
                source.close()
                with self._lock:
                    self._hands, self._primary = [], None
            self._stop.wait(_RETRY_S)
        if detector is not None:
            detector.close()

    detect_error: str | None = None

    # ---------- 检测后处理 ----------
    def _attach_depth(self, hands: list[Hand], depth: np.ndarray, shape) -> None:
        h, w = shape[:2]
        for hand in hands:
            u, v = hand.cx * w, hand.cy * h
            d = depth_at(depth, u, v)
            if d is not None:
                hand.xyz = deproject(self.intrinsics, u, v, d)

    # 换手的滞回：别的手要比当前锁定的手好出这个倍数，并持续这么久，才换过去
    SWITCH_RATIO = 1.5
    SWITCH_HOLD_S = 0.5

    def _rank(self, hand: Hand) -> float:
        """选手用的评分，越大越好。深度相机按远近（越近越好），普通相机按视野质量。"""
        if hand.xyz is not None:
            return 1.0 / max(float(hand.xyz[2]), 0.05)
        return view_quality(hand)

    def _update_primary(self, hands: list[Hand], now: float) -> None:
        """在本相机视野里选出最优的一只手并锁定。

        已锁定时按画面位置就近延续同一只手，避免在两只差不多的手之间来回跳；
        只有另一只手明显更好并持续一小段时间，或锁定的手消失了，才换。
        """
        if self._depth is not None:
            hands = [h for h in hands if h.xyz is not None]
        if not hands:
            if now - self._primary_time > 0.4:
                self._primary = None
            self._challenger_since = None
            return
        best = max(hands, key=self._rank)
        prev = self._primary
        if prev is not None:
            same = min(hands, key=lambda h: (h.cx - prev.cx) ** 2 + (h.cy - prev.cy) ** 2)
            if np.hypot(same.cx - prev.cx, same.cy - prev.cy) < 0.25:
                if best is not same and self._rank(best) > self._rank(same) * self.SWITCH_RATIO:
                    if self._challenger_since is None:
                        self._challenger_since = now
                    if now - self._challenger_since >= self.SWITCH_HOLD_S:
                        self._challenger_since = None
                        self._primary, self._primary_time = best, now
                        return
                else:
                    self._challenger_since = None
                self._primary, self._primary_time = same, now
                return
        self._challenger_since = None
        self._primary, self._primary_time = best, now

    _challenger_since: float | None = None

    # ---------- 对外读取 ----------
    def primary_hand(self, max_age_s: float = 0.4) -> Hand | None:
        with self._lock:
            if self._primary is None or time.monotonic() - self._primary_time > max_age_s:
                return None
            return self._primary

    def hand_count(self) -> int:
        with self._lock:
            return len(self._hands)

    def latest(self) -> tuple[np.ndarray, np.ndarray | None, float] | None:
        """最新一帧的 (彩色, 深度/米, 采集时刻) 副本；还没有画面时返回 None。"""
        with self._lock:
            if self._frame is None:
                return None
            depth = None if self._depth is None else self._depth.copy()
            return self._frame.copy(), depth, self._frame_time

    def point_at(self, u_norm: float, v_norm: float) -> np.ndarray | None:
        """归一化像素处的 3D 点（相机光学系）。仅深度相机可用，标定时使用。"""
        with self._lock:
            depth, intr = self._depth, self.intrinsics
        if depth is None or intr is None:
            return None
        h, w = depth.shape
        d = depth_at(depth, u_norm * w, v_norm * h, radius=3)
        return None if d is None else deproject(intr, u_norm * w, v_norm * h, d)

    def jpeg(self, quality: int = 70) -> bytes:
        with self._lock:
            frame = None if self._frame is None else self._frame.copy()
            hands, primary = list(self._hands), self._primary
            stale = time.monotonic() - self._frame_time > 1.0
        if frame is None or stale or self.error:
            frame = np.full((self.cfg.height, self.cfg.width, 3), 28, np.uint8)
            # 具体原因由前端用中文显示（OpenCV 画不了中文），这里只画一个居中的占位标记
            size = cv2.getTextSize("NO SIGNAL", cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)[0]
            origin = ((frame.shape[1] - size[0]) // 2, (frame.shape[0] + size[1]) // 2)
            cv2.putText(frame, "NO SIGNAL", origin, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (90, 90, 110), 2)
        else:
            self._annotate(frame, hands, primary)
        if frame.shape[1] > 800:  # 网页预览不需要全分辨率，缩小后编码更省 CPU 和带宽
            scale = 640 / frame.shape[1]
            frame = cv2.resize(frame, (640, int(round(frame.shape[0] * scale))), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes() if ok else b""

    def _annotate(self, frame: np.ndarray, hands: list[Hand], primary: Hand | None) -> None:
        h, w = frame.shape[:2]
        k = max(1, round(w / 640))  # 线宽随画面大小缩放
        cx, cy = w // 2, h // 2
        if self.deadzone > 0:
            dx, dy = int(self.deadzone * w / 2), int(self.deadzone * h / 2)
            cv2.rectangle(frame, (cx - dx, cy - dy), (cx + dx, cy + dy), (200, 200, 200), k)
        cv2.drawMarker(frame, (cx, cy), (200, 200, 200), cv2.MARKER_CROSS, 14 * k, k)
        for hand in hands:
            is_primary = hand is primary
            color = (80, 220, 120) if is_primary else (150, 150, 150)
            pts = (hand.landmarks * [w, h]).astype(int)
            for a, b in HAND_CONNECTIONS:
                cv2.line(frame, tuple(pts[a]), tuple(pts[b]), color, (2 if is_primary else 1) * k)
            center = (int(hand.cx * w), int(hand.cy * h))
            cv2.circle(frame, center, 6 * k, color, -1)
            if is_primary and self.overlay_active:
                cv2.line(frame, (cx, cy), center, (60, 180, 255), 2 * k)
            if hand.xyz is not None:
                cv2.putText(frame, f"{hand.xyz[2]:.2f} m", (center[0] + 10, center[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        if self.overlay_text:
            cv2.putText(frame, self.overlay_text, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 180, 255), 2)
