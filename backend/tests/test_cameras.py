"""相机线程里“在视野中选最优的一只手并锁定”的逻辑。不打开任何相机。"""

import numpy as np

from soarm.config import CameraConfig, DetectorConfig
from soarm.perception.cameras import CameraWorker
from soarm.perception.hand import Hand, view_quality


def _hand(cx, cy, size=0.4, score=0.9, depth=None):
    xyz = None if depth is None else np.array([0.0, 0.0, depth])
    return Hand(cx, cy, size, score, np.zeros((21, 2), np.float32), xyz)


def _worker():
    return CameraWorker("cam", CameraConfig(device="0"), DetectorConfig(), lambda: None)


def test_view_quality_prefers_centred_large_confident_hands():
    centre = view_quality(_hand(0.5, 0.5))
    assert centre > view_quality(_hand(0.9, 0.9))
    assert centre > view_quality(_hand(0.5, 0.5, size=0.1))
    assert centre > view_quality(_hand(0.5, 0.5, score=0.55))
    assert 0.0 < view_quality(_hand(1.0, 1.0, size=0.05, score=0.5)) < 0.2


def test_picks_the_best_hand_in_view():
    w = _worker()
    far_corner, good = _hand(0.9, 0.85, size=0.15), _hand(0.45, 0.5, size=0.4)
    w._update_primary([far_corner, good], 1.0)
    assert w._primary is good


def test_keeps_the_locked_hand_when_another_is_only_slightly_better():
    w = _worker()
    a = _hand(0.3, 0.5, size=0.35)
    w._update_primary([a], 1.0)
    for i in range(40):  # 另一只手略好一点，持续很久也不换
        a2, b = _hand(0.3, 0.5, size=0.35), _hand(0.6, 0.5, size=0.38)
        w._update_primary([a2, b], 1.0 + i * 0.033)
    assert w._primary is a2


def test_switches_when_another_hand_is_clearly_better_for_a_while():
    w = _worker()
    w._update_primary([_hand(0.88, 0.85, size=0.12, score=0.6)], 1.0)
    t = 1.0
    for _ in range(6):  # 约 0.2 秒：还没到换手的持续时间
        t += 0.033
        poor, great = _hand(0.88, 0.85, size=0.12, score=0.6), _hand(0.5, 0.5, size=0.45)
        w._update_primary([poor, great], t)
    assert w._primary is poor
    for _ in range(15):  # 再过约 0.5 秒：换到明显更好的那只
        t += 0.033
        poor, great = _hand(0.88, 0.85, size=0.12, score=0.6), _hand(0.5, 0.5, size=0.45)
        w._update_primary([poor, great], t)
    assert w._primary is great


def test_moves_to_remaining_hand_when_locked_one_disappears():
    w = _worker()
    a, b = _hand(0.3, 0.5, size=0.4), _hand(0.8, 0.5, size=0.2)
    w._update_primary([a, b], 1.0)
    assert w._primary is a
    b2 = _hand(0.8, 0.5, size=0.2)
    w._update_primary([b2], 1.033)
    assert w._primary is b2


def test_follows_the_same_hand_as_it_moves_past_another():
    w = _worker()
    t = 1.0
    w._update_primary([_hand(0.2, 0.5), _hand(0.8, 0.5, size=0.38)], t)
    for i in range(1, 12):  # 锁定的手从左往右移动，经过另一只差不多的手
        t += 0.033
        moving, still = _hand(0.2 + i * 0.04, 0.5), _hand(0.8, 0.5, size=0.38)
        w._update_primary([moving, still], t)
        assert w._primary is moving


def test_depth_camera_picks_the_nearest_hand():
    w = _worker()
    w._depth = np.zeros((4, 4), np.float32)  # 标记为深度相机
    near, far = _hand(0.9, 0.9, size=0.1, depth=0.4), _hand(0.5, 0.5, size=0.4, depth=1.2)
    w._update_primary([far, near, _hand(0.5, 0.5)], 1.0)  # 没有深度的手不参与
    assert w._primary is near
