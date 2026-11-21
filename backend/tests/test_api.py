"""通过 HTTP / WebSocket 走一遍控制台接口。使用模拟臂，不配置任何相机，不接触硬件。"""

import time

import numpy as np
import pytest
from fastapi.testclient import TestClient

from soarm.app import create_app
from soarm.config import Config
from soarm.perception.hand import make_detector


@pytest.fixture
def client(tmp_path):
    cfg = Config(mock=True, cameras={}, calibration_dir=str(tmp_path))
    with TestClient(create_app(cfg)) as c:
        yield c


def wait_for(client, predicate, timeout=12.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = client.get("/api/state").json()
        if predicate(state):
            return state
        time.sleep(0.1)
    raise AssertionError(f"等待超时，最后状态: {state['mode']} {state['arms']['left']}")


def test_static_frontend_is_served(client):
    assert "SO-ARM" in client.get("/").text
    assert client.get("/static/app.js").status_code == 200
    assert client.get("/static/style.css").status_code == 200


def test_meta_lists_keymaps(client):
    meta = client.get("/api/meta").json()
    assert set(meta["keymap"]) == {"left", "right"}
    assert meta["keymap"]["left"]["KeyW"] == ["x", 1]
    assert not set(meta["keymap"]["left"]) & set(meta["keymap"]["right"])  # 两臂键位不重叠
    assert set(meta["head_keymap"]) == {"ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown"}


def test_mode_switch_and_keyboard_over_websocket(client):
    assert client.post("/api/mode", json={"mode": "keyboard"}).status_code == 200
    state = wait_for(client, lambda s: s["arms"]["left"]["status"] in ("待命", "已到位"))
    x0 = state["arms"]["left"]["tip"][0]
    with client.websocket_connect("/ws") as ws:
        assert ws.receive_json()["mode"] == "keyboard"
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            ws.send_json({"keys": ["KeyW"]})
            time.sleep(0.05)
    # 连接断开即视为松开
    time.sleep(0.3)
    state = client.get("/api/state").json()
    assert state["keys"] == []
    assert state["arms"]["left"]["tip"][0] > x0 + 0.02


def test_estop_blocks_mode_change(client):
    assert client.post("/api/estop", json={"active": True}).status_code == 200
    res = client.post("/api/mode", json={"mode": "follow"})
    assert res.status_code == 409 and "急停" in res.json()["detail"]
    assert client.post("/api/go", json={"pose": "home"}).status_code == 409
    client.post("/api/estop", json={"active": False})
    assert client.post("/api/mode", json={"mode": "follow"}).status_code == 200


def test_unknown_mode_and_camera_are_rejected(client):
    assert client.post("/api/mode", json={"mode": "dance"}).status_code == 409
    assert client.get("/stream/nope").status_code == 404


def test_calibration_requires_depth_camera(client):
    res = client.post("/api/calib/sample", json={"arm": "left", "u": 0.5, "v": 0.5})
    assert res.status_code == 409
    assert client.post("/api/calib/solve", json={"arm": "left"}).status_code == 409


def test_mediapipe_detector_loads_and_handles_empty_frame():
    cfg = Config()
    model = cfg.path(cfg.detector.model_path)
    if not model.exists():
        pytest.skip("未下载手部模型，先运行 scripts/fetch_models.sh")
    detector = make_detector("mediapipe", model, 2, 0.5)
    try:
        assert detector.detect(np.zeros((480, 640, 3), np.uint8), 1) == []
        assert detector.detect(np.zeros((480, 640, 3), np.uint8), 1) == []  # 时间戳重复也不报错
    finally:
        detector.close()


def test_start_mode_is_entered_automatically(tmp_path):
    cfg = Config(mock=True, cameras={}, calibration_dir=str(tmp_path), start_mode="follow")
    with TestClient(create_app(cfg)) as c:
        assert c.get("/api/state").json()["mode"] == "follow"


def test_disabled_camera_is_never_opened(tmp_path):
    from soarm.config import CameraConfig

    cfg = Config(
        mock=True,
        cameras={"head": CameraConfig(enabled=False, type="realsense")},
        calibration_dir=str(tmp_path),
    )
    with TestClient(create_app(cfg)) as c:
        cam = c.get("/api/state").json()["cameras"]["head"]
        assert "停用" in cam["error"] and cam["fps"] == 0
        assert not c.app.state.controller.cameras["head"]._thread.is_alive()
