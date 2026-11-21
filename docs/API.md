# 接口

后端默认监听 `127.0.0.1:8000`。没有鉴权，只应在本机或可信内网使用。

可预期的操作错误（急停中、未连接、采样点不足等）返回 `409`，响应体为 `{"detail": "原因"}`，前端直接显示给操作者。

## REST

| 方法与路径 | 请求体 | 说明 |
|---|---|---|
| `GET /api/state` | – | 当前状态快照，结构同 WebSocket 下行 |
| `GET /api/meta` | – | 模式列表、两臂与云台的键位表、相机名、臂与相机的对应关系 |
| `POST /api/mode` | `{"mode": "idle\|keyboard\|follow\|fused"}` | 切换模式。进入 follow / fused 会先回初始位。`fused` 默认是指向式深度引导 |
| `POST /api/estop` | `{"active": true\|false}` | 急停 / 解除。两者都会把模式置为 idle |
| `POST /api/go` | `{"pose": "home\|rest", "release": false}` | 前往初始位或休息位；`release` 为真时到位后释放力矩 |
| `POST /api/torque` | `{"active": true\|false}` | 开关所有臂的力矩。关闭后手臂会下落 |
| `POST /api/arm/enable` | `{"arm": "left", "enabled": false}` | 停用或启用一只臂。停用后它在任何模式下都只保持当前位置 |
| `POST /api/arm/lend` | `{"arm": "right"}` | 把一只臂借给开发程序：停止指挥并松开串口，舵机力矩不变 |
| `POST /api/restore` | – | 恢复全量展示：让登记在案的开发程序退出，接管所有借出的臂。返回 `{"evicted", "restored", "failed"}` |
| `POST /api/reconnect` | – | 重新连接未连上的机械臂和云台 |
| `POST /api/head/preset` | – | 云台回到标定时的姿态 |
| `POST /api/calib/sample` | `{"arm": "left", "u": 0.5, "v": 0.5}` | 记录一个标定点。`u`、`v` 是深度相机画面上夹爪尖的归一化坐标 |
| `POST /api/calib/undo` | `{"arm": "left"}` | 撤销该臂最后一个标定点 |
| `POST /api/calib/clear` | `{"arm": "left"}` | 清空该臂的标定点 |
| `POST /api/calib/auto/solve` | `{"arm": "left"}` | 用自动积累的配对样本求解该臂外参并保存，返回 `{"arm", "rms", "samples", "collected"}` |
| `POST /api/calib/auto/clear` | `{"arm": "left"}` | 清空该臂自动积累的样本 |
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
      "enabled": true,
      "lent": false,
      "error": null,
      "torque": true,
      "status": "移动中",
      "joints": {"shoulder_pan": 0.0, "shoulder_lift": -75.0, "elbow_flex": 60.0,
                 "wrist_flex": 10.0, "wrist_roll": -90.0, "gripper": 0.0},
      "limits": {"shoulder_pan": [-46.6, 27.5], "...": "该臂生效的关节限位"},
      "target": {"...": "同 joints，为已下发的目标"},
      "tip": [0.256, 0.0, 0.219],
      "pitch": 0.4,
      "moving_to": null,
      "calibrated": false,
      "calib_rms_mm": null,
      "calib_samples": 0,
      "fused_selected": false,
      "auto_samples": 0,
      "view_quality": 0.0
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
                   "detect_fps": 30.0, "read_ms": 33.5, "detect_ms": 12.5, "size": [1280, 720],
                   "hands": 1, "locked": true, "depth": false,
                   "primary": {"cx": 0.52, "cy": 0.4, "size": 0.6, "score": 0.95, "xyz": null}}
  }
}
```

字段说明：

- 关节角单位为度，`gripper` 为开合度 0–100。
- `tip` 是夹爪参考点在该臂基座系下的位置（米，x 前 y 左 z 上），`pitch` 是夹爪俯仰角（度，抬头为正）。
- `moving_to` 在回初始位或收到休息位的途中为 `"home"` 或 `"rest"`。
- `calib_rms_mm` 是已保存外参的残差；`calib_samples` 是尚未求解的暂存点数。
- `fused_selected` 表示这只臂当前正在跟一只手：腕部跟随模式下两只臂可以同时为真；深度引导模式下是被选去够手的那只。
- `auto_samples` 是外参自动标定已为这只臂积累的配对样本数。
- `view_quality` 是这只臂的腕部相机当前看手的质量评分，0–1，0 表示看不到。
- `cameras.*.fps` / `detect_fps` 是采集帧率和检测频率；`read_ms` 是取一帧的平均耗时（含等待下一帧），`detect_ms` 是单帧检测的平均耗时，用于排查卡顿。当前模式用不到某路相机的检测时，它的 `detect_fps` 为 0。
- `lent` 为真表示这只臂已借给开发程序，此时 `connected` 为假。
- 顶层的 `guidance_hits` 是最近若干次“引导命中”：深度引导把臂带过去后，腕部相机第一眼看到手时，手在腕部画面里的位置 `{arm, time, cx, cy}`。离画面中心越近说明引导越准。
- 顶层的 `floor_pitch_deg` 是最近一次由地面测得的深度相机低头角度，没测到时为 `null`。
- `head` 在未配置云台时为 `null`。
- `cameras.*.primary` 是当前锁定的主手：`cx`、`cy` 为掌心的归一化画面坐标，`size` 为手在画面中的相对大小，`xyz` 为相机光学系下的 3D 坐标（米，x 右 y 下 z 前），只有深度相机有。没有主手时为 `null`。

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
