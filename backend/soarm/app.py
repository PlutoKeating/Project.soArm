"""Web 服务：REST 命令 + WebSocket 状态/按键 + MJPEG 视频流 + 静态前端。"""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
from contextlib import asynccontextmanager
from functools import partial

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from . import lab
from .config import REPO_ROOT, Config, load_config
from .control.controller import HEAD_KEYMAP, KEYMAP, MODES, Controller
from .perception.cameras import CameraWorker, RemoteHandCamera
from .perception.hand import make_detector
from .robot.factory import make_robot

logger = logging.getLogger(__name__)
FRONTEND_DIR = REPO_ROOT / "frontend"
STATE_HZ = 15
STREAM_FPS = 15


class ModeBody(BaseModel):
    mode: str


class FlagBody(BaseModel):
    active: bool


class GoBody(BaseModel):
    pose: str
    release: bool = False


class CalibSampleBody(BaseModel):
    arm: str
    u: float
    v: float


class ArmBody(BaseModel):
    arm: str


class ControllerBody(BaseModel):
    controller: str


class ArmEnableBody(BaseModel):
    arm: str
    enabled: bool


def build_controller(cfg: Config) -> Controller:
    detector_factory = partial(
        make_detector,
        cfg.detector.backend,
        cfg.path(cfg.detector.model_path),
        cfg.detector.max_hands,
        cfg.detector.min_confidence,
    )
    cameras = {
        name: RemoteHandCamera(name, cam_cfg) if cam_cfg.type == "remote"
        else CameraWorker(name, cam_cfg, cfg.detector, detector_factory)
        for name, cam_cfg in cfg.cameras.items()
    }
    arms, head = make_robot(cfg)
    return Controller(cfg, arms, head, cameras)


def create_app(cfg: Config) -> FastAPI:
    controller = build_controller(cfg)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        if cfg.lab:
            lab.pidfile().write_text(str(os.getpid()))  # 登记为开发试验进程，生产恢复时会让它退出
        for cam in controller.cameras.values():
            cam.start()
        await asyncio.to_thread(controller.start)
        if cfg.start_mode != "idle":
            try:
                controller.set_mode(cfg.start_mode)
                logger.info("已自动进入模式 %s", cfg.start_mode)
            except (RuntimeError, ValueError) as e:
                logger.error("无法自动进入模式 %s: %s", cfg.start_mode, e)
        yield
        if cfg.lab:
            # 开发试验进程被要求让路：立刻松开串口，不收手、不动力矩，生产会接管
            await asyncio.to_thread(controller.abort)
            lab.release()
        else:
            await asyncio.to_thread(controller.shutdown)
        for cam in controller.cameras.values():
            cam.stop()

    app = FastAPI(title="SO-ARM 演示控制台", lifespan=lifespan)
    app.state.controller = controller

    def command(fn, *args, **kwargs):
        """执行控制命令，把可预期的错误转成 409，前端直接显示给操作者。"""
        try:
            return fn(*args, **kwargs)
        except (RuntimeError, ValueError, KeyError) as e:
            raise HTTPException(status_code=409, detail=str(e)) from e

    @app.get("/api/state")
    def get_state():
        return controller.snapshot()

    @app.get("/api/meta")
    def get_meta():
        return {
            "modes": MODES,
            "keymap": {arm: {code: list(v) for code, v in keys.items()} for arm, keys in KEYMAP.items()},
            "head_keymap": {code: list(v) for code, v in HEAD_KEYMAP.items()},
            "cameras": list(cfg.cameras),
            "head_camera": cfg.head_camera,
            "arms": {name: {"camera": a.camera} for name, a in cfg.arms.items()},
        }

    @app.post("/api/mode")
    def post_mode(body: ModeBody):
        command(controller.set_mode, body.mode)
        return {"ok": True}

    @app.post("/api/estop")
    def post_estop(body: FlagBody):
        controller.set_estop(body.active)
        return {"ok": True}

    @app.post("/api/go")
    def post_go(body: GoBody):
        command(controller.go, body.pose, body.release)
        return {"ok": True}

    @app.post("/api/torque")
    def post_torque(body: FlagBody):
        command(controller.set_torque, body.active)
        return {"ok": True}

    @app.post("/api/arm/enable")
    def post_arm_enable(body: ArmEnableBody):
        command(controller.set_arm_enabled, body.arm, body.enabled)
        return {"ok": True}

    @app.post("/api/follow/controller")
    def post_follow_controller(body: ControllerBody):
        command(controller.set_follow_controller, body.controller)
        return {"ok": True}

    @app.post("/api/arm/lend")
    def post_arm_lend(body: ArmBody):
        command(controller.lend_arm, body.arm)
        return {"ok": True}

    @app.post("/api/restore")
    async def post_restore():
        return await asyncio.to_thread(controller.restore_all)

    @app.post("/api/reconnect")
    async def post_reconnect():
        await asyncio.to_thread(controller.connect_all)
        return {"ok": True}

    @app.post("/api/head/preset")
    def post_head_preset():
        controller.head_to_preset()
        return {"ok": True}

    @app.post("/api/calib/sample")
    def post_calib_sample(body: CalibSampleBody):
        return command(controller.calib_add, body.arm, body.u, body.v)

    @app.post("/api/calib/undo")
    def post_calib_undo(body: ArmBody):
        return {"count": controller.calib_undo(body.arm)}

    @app.post("/api/calib/clear")
    def post_calib_clear(body: ArmBody):
        controller.calib_clear(body.arm)
        return {"ok": True}

    @app.post("/api/calib/solve")
    def post_calib_solve(body: ArmBody):
        return command(controller.calib_solve, body.arm)

    @app.post("/api/calib/auto/solve")
    def post_calib_auto_solve(body: ArmBody):
        return command(controller.calib_auto_solve, body.arm)

    @app.post("/api/calib/auto/clear")
    def post_calib_auto_clear(body: ArmBody):
        controller.calib_auto_clear(body.arm)
        return {"ok": True}

    @app.get("/stream/{name}")
    async def stream(name: str):
        cam = controller.cameras.get(name)
        if cam is None:
            raise HTTPException(status_code=404, detail=f"没有相机 {name}")

        async def frames():
            while True:
                jpg = await asyncio.to_thread(cam.jpeg)
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
                await asyncio.sleep(1 / STREAM_FPS)

        return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.websocket("/ws")
    async def ws(websocket: WebSocket):
        """下行：周期性状态快照。上行：{"keys": [...]} 按键心跳。断开即视为全部松开。"""
        await websocket.accept()

        async def push_state():
            while True:
                await websocket.send_json(controller.snapshot())
                await asyncio.sleep(1 / STATE_HZ)

        pusher = asyncio.create_task(push_state())
        try:
            while True:
                msg = await websocket.receive_json()
                if "keys" in msg:
                    controller.set_keys([str(k) for k in msg["keys"]][:16])
        except WebSocketDisconnect:
            pass
        finally:
            controller.set_keys([])
            pusher.cancel()

    @app.get("/")
    def index():
        return FileResponse(FRONTEND_DIR / "index.html")

    app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")
    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="SO-ARM 双臂演示控制台")
    parser.add_argument("--config", help="配置文件路径，默认 config/config.yaml")
    parser.add_argument("--mock", action="store_true", help="不连接机械臂和云台，用模拟替身")
    parser.add_argument("--mode", choices=MODES, help="启动后自动进入的模式，覆盖配置里的 start_mode")
    parser.add_argument("--lab", action="store_true", help="作为开发试验进程运行：登记进程号，被要求让路时立刻松开机械臂")
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    cfg = load_config(args.config)
    if args.mock:
        cfg.mock = True
    if args.mode:
        cfg.start_mode = args.mode
    if args.lab:
        cfg.lab = True
    host, port = args.host or cfg.server.host, args.port or cfg.server.port
    logger.info("控制台地址 http://%s:%d  （mock=%s）", host, port, cfg.mock)
    # 视频流和 WebSocket 是长连接，浏览器开着时不会自己断开。不设超时的话 Ctrl+C 后
    # 服务会一直等它们，走不到“先收到休息位再释放力矩”的退出流程
    uvicorn.run(create_app(cfg), host=host, port=port, log_level="warning", timeout_graceful_shutdown=2)


if __name__ == "__main__":
    main()
