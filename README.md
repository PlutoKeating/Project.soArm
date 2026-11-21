# SO-ARM 双臂演示控制台

面向一套双 SO-100/SO-101 机械臂演示套件的浏览器控制台。套件包含：

- 两只 SO-100/101 follower 机械臂，夹爪上各有一个 UVC 腕部相机
- 一台 Intel RealSense D435 系列深度相机，装在 2 自由度云台上
- 三块 Waveshare Bus Servo Adapter (A) 控制板（左臂、右臂、云台各一块），舵机为 Feetech STS3215

## 能做什么

| 模式 | 快捷键 | 说明 |
|---|---|---|
| 待机 | `0` | 保持当前位置 |
| 键盘控制 | `1` | 末端笛卡尔控制，左右两套键位同时控制两臂，方向键控制云台。按住才动，松开即停 |
| 腕部跟随 | `2` | 腕部相机看到手就转动对准它（只转动、不调距离），看不到手 1.5 秒后回初始位。两只臂同一时刻至多一只在跟，由视野更好的那只负责 |
| 深度引导 | `3` | 深度相机在大视野里定位手的 3D 位置，把合适的那只臂的腕部相机转过去对准，手进入腕部视野后由腕部相机接管 |

任何时候按空格急停。面向展会场景，默认带限速、工作空间限制、两臂之间相机中柱的避让、受阻让位和按键心跳超时保护。每只臂可以单独停用。

## 目录

```text
backend/soarm/     后端：控制循环、运动学、手部检测、硬件驱动、Web 服务
backend/tests/     不依赖硬件的测试（模拟臂 + 假相机）
frontend/          前端：单页控制台，无构建步骤
config/            配置模板；本机的 config.yaml 不入库
scripts/           下载手部模型、下载社区参考源码
docs/              全局文档
third_party/       参考源码（脚本下载，不入库）
models/            手部模型（脚本下载，不入库）
calibration/       深度相机外参标定结果（运行时生成，不入库）
```

## 快速开始

```bash
uv sync --extra realsense        # 安装依赖（Python 3.12）
scripts/fetch_models.sh          # 下载 MediaPipe 手部模型
uv run soarm-console --mock      # 不接硬件，用模拟臂跑通控制台
```

打开 <http://127.0.0.1:8000>。展示时用 `scripts/production.sh start` 常驻运行。接真机的完整步骤（扫描串口、校准、相机、外参标定）见 [docs/QUICK_START.md](docs/QUICK_START.md)。

## 文档

- [docs/QUICK_START.md](docs/QUICK_START.md)：环境安装与真机上电流程
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)：整体架构与关键设计决定
- [docs/API.md](docs/API.md)：HTTP / WebSocket 接口
- [docs/REFERENCES.md](docs/REFERENCES.md)：调研过的社区与官方方案
- [docs/devices/](docs/devices/)：设备检测报告与实测记录
- [backend/docs/README.md](backend/docs/README.md)、[frontend/docs/README.md](frontend/docs/README.md)：模块文档

## 当前状态

- 控制逻辑在模拟臂上通过全部自动化测试。
- 三台相机已实测：三路画面、手部检测和深度读数都正常。
- 两只机械臂已上电实测：展开到初始位、腕部跟随（方向、回位、双臂择优单臂跟随）、中柱避让、退出时自动收手都已验证。
- 云台已接入并设好零点。
- **尚未验证**：键盘笛卡尔控制在真机上的各个方向；深度引导和外参自动标定只在合成数据上测过，真机上卡在深度相机朝向（现在朝下看地面）。

实测记录见 [docs/devices/VERIFIED.md](docs/devices/VERIFIED.md)。
