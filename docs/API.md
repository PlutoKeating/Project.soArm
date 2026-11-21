# 接口

后端默认监听 `127.0.0.1:8000`。没有鉴权，只应在本机或可信内网使用。

可预期的操作错误（急停中、未连接、采样点不足等）返回 `409`，响应体为 `{"detail": "原因"}`，前端直接显示给操作者。

## REST

| 方法与路径 | 请求体 | 说明 |
|---|---|---|
| `GET /api/state` | – | 当前状态快照，结构同 WebSocket 下行 |
| `GET /api/meta` | – | 模式列表、两臂与云台的键位表、相机名、臂与相机的对应关系 |
| `POST /api/mode` | `{"mode": "idle\|keyboard\|follow\|fused"}` | 切换模式。进入 follow / fused 会先回初始位 |
| `POST /api/estop` | `{"active": true\|false}` | 急停 / 解除。两者都会把模式置为 idle |
| `POST /api/go` | `{"pose": "home\|rest", "release": false}` | 前往初始位或休息位；`release` 为真时到位后释放力矩 |
| `POST /api/torque` | `{"active": true\|false}` | 开关所有臂的力矩。关闭后手臂会下落 |
| `POST /api/reconnect` | – | 重新连接未连上的机械臂和云台 |
| `POST /api/head/preset` | – | 云台回到标定时的姿态 |
| `POST /api/calib/sample` | `{"arm": "left", "u": 0.5, "v": 0.5}` | 记录一个标定点。`u`、`v` 是深度相机画面上夹爪尖的归一化坐标 |
| `POST /api/calib/undo` | `{"arm": "left"}` | 撤销该臂最后一个标定点 |
| `POST /api/calib/clear` | `{"arm": "left"}` | 清空该臂的标定点 |
| `POST /api/calib/solve` | `{"arm": "left"}` | 求解外参并保存，返回 `{"arm", "rms", "samples"}`，`rms` 单位为米 |

## 视频

`GET /stream/{camera}`：MJPEG 流（`multipart/x-mixed-replace`），约 15 fps，画面已叠加手部骨架、主手、死区框。相机离线时输出占位画面。相机名不存在返回 `404`。

## WebSocket `/ws`

**上行**（浏览器 → 后端）：按键心跳。

```json
{"keys": ["KeyW", "KeyJ"]}
```

`keys` 是当前按住的运动键，使用 `KeyboardEvent.code`。前端每 100 ms 发送一次，按键变化时立即发送。后端超过 0.35 秒没收到就视为全部松开；连接断开时立即清空。

**下行**（后端 → 浏览器）：约 15 Hz 的状态快照。

```json
{
  "mode": "keyboard",
  "estop": false,
  "mock": true,
  "loop_hz": 30.0,
  "keys": ["KeyW"],
  "message": "",
  "head_camera": "head",
  "arms": {
    "left": {
      "connected": true,
      "error": null,
      "torque": true,
      "status": "移动中",
      "joints": {"shoulder_pan": 0.0, "shoulder_lift": -75.0, "elbow_flex": 60.0,
                 "wrist_flex": 10.0, "wrist_roll": 0.0, "gripper": 0.0},
      "target": {"...": "同 joints，为已下发的目标"},
      "tip": [0.256, 0.0, 0.219],
      "pitch": 0.4,
      "moving_to": null,
      "calibrated": false,
      "calib_rms_mm": null,
      "calib_samples": 0,
      "fused_selected": false
    }
  },
  "head": {
    "connected": true,
    "error": null,
    "pos": {"pan": 0.0, "tilt": 0.0},
    "preset": null,
    "matches_preset": true
  },
  "cameras": {
    "wrist_left": {"error": null, "detect_error": null, "fps": 30.0,
                   "detect_fps": 20.0, "hands": 1, "locked": true, "depth": false}
  }
}
```

字段说明：

- 关节角单位为度，`gripper` 为开合度 0–100。
- `tip` 是夹爪参考点在该臂基座系下的位置（米，x 前 y 左 z 上），`pitch` 是夹爪俯仰角（度，抬头为正）。
- `moving_to` 在回初始位或收到休息位的途中为 `"home"` 或 `"rest"`。
- `calib_rms_mm` 是已保存外参的残差；`calib_samples` 是尚未求解的暂存点数。
- `head` 在未配置云台时为 `null`。

## 键位

以 `GET /api/meta` 返回的为准，默认如下。

| 动作 | 左臂 | 右臂 |
|---|---|---|
| 前 / 后 | `W` / `S` | `I` / `K` |
| 左 / 右 | `A` / `D` | `J` / `L` |
| 上 / 下 | `R` / `F` | `Y` / `H` |
| 抬头 / 低头 | `Q` / `E` | `U` / `O` |
| 腕部旋转 | `Z` / `X` | `N` / `M` |
| 夹爪 开 / 合 | `V` / `C` | `.` / `,` |

云台：方向键。全局：空格急停，`0`–`3` 切换模式，`G` 回初始位。
