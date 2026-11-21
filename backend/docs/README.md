# 后端

Python 包 `soarm`，位于 `backend/soarm/`。依赖由仓库根目录的 `pyproject.toml` 管理。

## 入口

| 命令 | 作用 |
|---|---|
| `uv run soarm-console [--mock] [--mode M] [--config PATH] [--host H] [--port P]` | 启动控制台（Web 服务 + 控制循环 + 相机线程）；`--mode` 指定启动后自动进入的模式 |
| `scripts/production.sh start\|stop\|status\|logs` | 以 systemd 用户服务常驻运行，异常退出自动拉起 |
| `scripts/lab.sh status\|idle\|lend\|restore\|deploy` | 展示与试验开发之间的快速切换 |
| `uv run soarm-tools scan` | 扫描串口上的 STS3215 舵机 |
| `uv run soarm-tools cameras` | 列出相机并保存快照 |
| `uv run soarm-tools watch <arm> [--once]` | 只读：显示该臂的行程限位并实时显示关节角度 |
| `uv run soarm-tools calibrate <arm>` | 运行 LeRobot 标准校准流程，结果写入舵机（一般不需要） |
| `uv run soarm-tools body-calib <arm>` | 用机械臂自身做手眼粗标定（不需要人手） |
| `scripts/trial.sh start\|stop\|status` | 借一只臂在本 worktree 里做真机试运行 |
| `uv run soarm-tools collect-calib [--prod URL]` | 旁路采集外参标定样本，不打断运行中的控制台 |
| `uv run soarm-tools solve-calib [样本文件]` | 用旁路采集的样本离线求解外参 |
| `uv run soarm-tools watchdog [--dry-run]` | 无人值守看门狗：异常时收到休息位、释放力矩并急停 |
| `uv run soarm-tools head-watch` | 释放云台力矩并显示原始刻度 |

## 文件

```text
soarm/
  app.py                  FastAPI 应用与命令行入口
  config.py               配置 dataclass 与 YAML 加载
  kinematics.py           URDF 正解 + 4 自由度逆解
  calib.py                触点法外参标定
  tools.py                上电准备工具
  lab.py                  生产与开发试验之间的交接（进程登记与让路）
  watchdog.py             无人值守看门狗的异常判定与安全停下动作
  assets/so101_new_calib.urdf   仅含运动学的 SO-101 URDF
  control/controller.py   模式状态机、三种模式、安全处理
  control/geometry.py     腕部相机视线与指向偏转量的几何换算
  perception/cameras.py   相机采集线程、主手锁定、标注
  perception/hand.py      手部检测接口与 MediaPipe 实现
  robot/base.py           Arm / Head 抽象接口
  robot/real.py           真机驱动（LeRobot Feetech 总线）
  robot/mock.py           模拟替身
  robot/factory.py        按配置创建真机或模拟对象
tests/                    不依赖硬件的测试
```

## 测试

```bash
uv run pytest
```

- `test_kinematics.py`：正逆解往返、初始位与休息位合理性、外参求解。
- `test_controller.py`：用模拟臂和假相机逐拍驱动控制器，覆盖三种模式与急停、限速、工作空间、受阻让位、单臂停用、中柱避让、两臂独立跟随与可选的单臂跟随。
- `test_cameras.py`：相机线程在视野里择优选手与锁定的逻辑。
- `test_depth_guidance.py`：视线几何、外参自动求解（含噪声与错配样本）、按真实几何成像的合成场景下的深度引导与自动标定。
- `test_watchdog.py`：看门狗的各项判定（含不该误报的情况），以及对着模拟控制台走一遍安全停下。
- `test_api.py`：经 HTTP / WebSocket 走一遍接口；检测器加载（未下载模型时跳过）。

测试不打开任何串口或相机。

## 配置

见 `config/config.example.yaml` 与 `soarm/config.py`。外参标定结果写入 `calibration/extrinsics.yaml`。机械臂的校准保存在舵机里，不使用校准文件。

架构说明见 [ARCHITECTURE.md](ARCHITECTURE.md)。
