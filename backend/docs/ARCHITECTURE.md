# 后端架构

全局视角见 [docs/ARCHITECTURE.md](../../docs/ARCHITECTURE.md)，这里只讲后端内部。

## 线程与锁

| 线程 | 来源 | 做什么 |
|---|---|---|
| 事件循环 | uvicorn | 处理 HTTP / WebSocket；视频帧编码放到线程池 |
| `control` | `Controller._run` | 按 `control_hz` 调用 `_tick` |
| `cam-<name>` | `CameraWorker._run` | 取帧、检测、更新主手 |

`Controller` 用一把可重入锁保护全部状态：`_tick` 和所有外部命令（切模式、急停、标定等）都在锁内执行，所以串口同一时刻只有一个使用者。`CameraWorker` 有自己的锁，只保护最新帧和检测结果。

## 控制循环一拍

```text
_tick
 ├─ 清理超时的按键
 ├─ fused 模式：_fused_select 决定由哪只臂去跟
 ├─ 每只已连接的臂：_tick_arm
 │    读实测位置
 │    力矩已释放 → 目标跟随实测，结束
 │    急停       → 重发冻结的目标，结束
 │    受阻检查   → 偏差过大则让位并暂停
 │    求期望关节角：goto(回位) > keyboard / follow / fused > 保持
 │    限速、限位后下发
 ├─ _tick_head：键盘模式下方向键调云台，其余保持
 └─ _update_overlays：告诉相机线程画面上该画什么
```

串口读写抛出异常时，该臂被标记为断开并记录错误，其它设备不受影响；通过 `POST /api/reconnect` 重连。

## 三种模式的状态

每只臂的运行时状态在 `ArmRuntime` 里：

- `cart`：当前的笛卡尔目标（键盘、深度引导用）。键盘模式下松开按键即清空，下次按键从当时的关节目标重新出发，所以不会累积漂移。
- `pan_off` / `tilt_off`：腕部对准产生的偏转量。腕部跟随时相对初始位，深度引导时相对引导姿态。
- `guided`：深度引导得到的关节角，同时作为下一拍逆解的种子。
- `goto`：正在前往的固定姿态（初始位或休息位），优先级高于模式逻辑。

切换模式、急停、回位都会调用 `_reset_motion` 清空这些状态。

## 主手锁定

`CameraWorker._update_primary`：没有锁定时选最近的手（深度相机按深度，腕部相机按画面中的大小）；已锁定时优先延续画面位置最接近的那只，直到它消失 0.4 秒以上。深度相机上没有有效深度的手不参与。

## 扩展点

- 换手部检测算法：实现 `HandDetector.detect`，在 `make_detector` 里注册，配置 `detector.backend`。
- 换机械臂驱动：实现 `robot/base.py` 的 `Arm`，在 `robot/factory.py` 里创建。
- 新增模式：在 `MODES` 里加名字，在 `_tick_arm` 的分支里返回期望关节角即可，安全处理自动生效。
