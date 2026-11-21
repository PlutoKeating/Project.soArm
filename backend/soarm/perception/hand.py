"""手部检测。控制层只依赖 Hand / HandDetector，换算法只需新增一个 HandDetector 子类。"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# MediaPipe 21 点手部骨架的连线
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20), (0, 17),
]  # fmt: skip
_PALM_POINTS = [0, 5, 9, 13, 17]


@dataclass
class Hand:
    cx: float  # 掌心在画面中的归一化坐标，0-1
    cy: float
    size: float  # 手部包围框对角线的归一化长度，越大说明越近
    score: float
    landmarks: np.ndarray  # (21, 2) 归一化坐标
    xyz: np.ndarray | None = field(default=None)  # 掌心在相机光学系下的 3D 坐标（米），仅深度相机有


class HandDetector(ABC):
    @abstractmethod
    def detect(self, frame_bgr: np.ndarray, timestamp_ms: int) -> list[Hand]: ...

    def close(self) -> None:  # noqa: B027
        pass


class MediaPipeDetector(HandDetector):
    """MediaPipe Hand Landmarker（Tasks API，VIDEO 模式）。每路相机一个实例。"""

    def __init__(self, model_path: Path, max_hands: int, min_confidence: float):
        import mediapipe as mp

        if not model_path.exists():
            raise FileNotFoundError(f"缺少手部模型 {model_path}，请先运行 scripts/fetch_models.sh")
        self._mp = mp
        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(model_path)),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=max_hands,
            min_hand_detection_confidence=min_confidence,
            min_hand_presence_confidence=min_confidence,
            min_tracking_confidence=min_confidence,
        )
        self._landmarker = mp.tasks.vision.HandLandmarker.create_from_options(options)
        self._last_ts = -1

    def detect(self, frame_bgr: np.ndarray, timestamp_ms: int) -> list[Hand]:
        # VIDEO 模式要求时间戳严格递增
        timestamp_ms = max(timestamp_ms, self._last_ts + 1)
        self._last_ts = timestamp_ms
        rgb = np.ascontiguousarray(frame_bgr[:, :, ::-1])
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)
        result = self._landmarker.detect_for_video(image, timestamp_ms)
        hands: list[Hand] = []
        for i, lms in enumerate(result.hand_landmarks):
            pts = np.array([[lm.x, lm.y] for lm in lms], dtype=np.float32)
            palm = pts[_PALM_POINTS].mean(axis=0)
            span = pts.max(axis=0) - pts.min(axis=0)
            score = result.handedness[i][0].score if result.handedness else 1.0
            hands.append(
                Hand(
                    cx=float(palm[0]),
                    cy=float(palm[1]),
                    size=float(np.hypot(span[0], span[1])),
                    score=float(score),
                    landmarks=pts,
                )
            )
        return hands

    def close(self) -> None:
        self._landmarker.close()


def make_detector(backend: str, model_path: Path, max_hands: int, min_confidence: float) -> HandDetector:
    if backend == "mediapipe":
        return MediaPipeDetector(model_path, max_hands, min_confidence)
    raise ValueError(f"未知的手部检测后端: {backend}")
