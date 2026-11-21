# 参考资料与调研结论

运行 `scripts/fetch_references.sh` 可把下列仓库下载到 `third_party/`（不入库）。调研时间：2026-11。

## 硬件判定

- 控制板是 Waveshare Bus Servo Adapter (A)，配 Feetech STS3215 总线舵机，是 SO-100/101 的标准配置。
- 两只臂都带夹爪和腕部相机，都是 follower，没有 leader。
- SO-100 与 SO-101 的 follower 在 LeRobot 里是同一个类，区别主要在走线和装配，型号不确定不影响软件。
- 整机形态（双臂 + 腕部相机 + 头部深度相机 + 2 自由度云台）与 XLeRobot 的上半身一致，只是没有底盘。

## 采用了什么

| 来源 | 采用的部分 |
|---|---|
| [huggingface/lerobot](https://github.com/huggingface/lerobot) 0.7.0 | Feetech 电机总线、校准流程（写入舵机的零点偏置与行程限位） |
| [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) | `so101_new_calib.urdf`，裁剪掉网格后放在 `backend/soarm/assets/`（Apache-2.0） |
| [Vector-Wangel/XLeRobot](https://github.com/Vector-Wangel/XLeRobot) | 双臂两套键位同时控制的交互方式；云台舵机挂在总线上的做法 |
| MediaPipe Hand Landmarker | 手部 21 关键点检测 |

## 参考但没有直接采用

| 来源 | 内容 | 没有采用的原因 |
|---|---|---|
| LeRobot 的 placo 逆解与键盘末端遥操作 | 完整 6 维位姿的软约束逆解 | SO-101 只有 5 个臂关节，改为解位置 + 俯仰的 4 维任务更稳，且少一个依赖 |
| XLeRobot `examples/` | 解析式 2 连杆逆解、YOLO 物体跟随 | 逆解带手调的关节补偿系数，依赖较旧的 LeRobot 版本 |
| [steven-tired/mediapipe-so101](https://github.com/steven-tired/mediapipe-so101) | 外部摄像头看操作者手腕，逆解遥操作 | 是遥操作，不是腕部相机跟随 |
| [MattiArlo/so101-hand-teleop](https://github.com/MattiArlo/so101-hand-teleop) | MediaPipe 双手遥操作 | 同上 |
| [henrynitzberg/SO101_Human_Teleop](https://github.com/henrynitzberg/SO101_Human_Teleop) | 双相机三角化手腕 3D 位置 | 我们有深度相机，不需要三角化 |
| [cal29292929/pai-so101-final](https://github.com/cal29292929/pai-so101-final) | 腕部 eye-in-hand 相机与视觉伺服 | 面向抓取物体 |
| [sowmiah08/hand_based_manipulation](https://github.com/sowmiah08/hand_based_manipulation) | 深度相机 + MediaPipe 指尖定位 | 基于 ROS 2 |
| [box2ai-robotics/lerobot-esp32](https://github.com/box2ai-robotics/lerobot-esp32) | 另一种键盘逆解控制实现 | 面向 ESP32 |

没有找到“腕部相机跟随人手”的现成开源实现，这部分控制环是本项目自己写的。

## 手部检测算法的选择

选 MediaPipe Hands：直接输出 21 个关键点，CPU 上即可实时，社区的 SO-101 手部项目几乎都用它。已知弱点是暗光和远距离小目标的检出率。检测器做成了可替换接口（`perception/hand.py` 的 `HandDetector`），效果不够时可以换 YOLO 手部权重或 RTMPose，不需要改控制代码。

## 相机实测结论

- 深度相机标签上的 “D435CB” 实际就是标准的 RealSense D435（无 IMU），SDK 识别正常。
- `pyrealsense2` 2.59 的 wheel 在 Ubuntu 26.04 上可直接使用，不需要编译，也不需要 udev 规则。
- 腕部相机是 OV9732 100° 广角 UVC 模组。

设备报告与实测记录见 [devices/](devices/)。
