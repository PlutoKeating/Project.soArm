# 深度摄像头 硬件识别与使用指南（RealSense D435 / D435CB）

> 识别时间：2026-11-21  
> 设备角色：SO-ARM / 具身智能双目主动红外立体深度摄像头（RGB-D）

---

## 1. 设备硬件身份（Hardware Identity）

| 属性字段 | 实测值 / 识别结果 |
| :--- | :--- |
| **USB Vendor ID (`idVendor`)** | `0x8086` (Intel Corp.) |
| **USB Product ID (`idProduct`)** | `0x0b07` (Intel® RealSense™ Depth Camera D435) |
| **固件 / 设备版本 (`bcdDevice`)** | `51.00` (USB 协议声明 `3.20`) |
| **厂商与产品字符串** | `Intel(R) RealSense(TM) Depth Camera 435` |
| **外壳贴纸型号 / S/N** | `Model: D435CB` / 外壳 S/N: `215322076692` |
| **芯片内部真实序列号 (`iSerial`)** | **`218423029391`** ⚠️ *(软件 SDK / LeRobot / ROS 2 调用时必须填此内部 ASIC 序列号！)* |
| **核心硬件架构** | Intel RealSense Vision Processor D4 + D430 全局快门双目红外模组 + OV2740 2MP RGB 传感器 + VCSEL 红外结构光投射器（无内置 IMU，区别于 D435i `8086:0b3a`） |
| **关于 `D435CB` 型号说明** | `D435C` / `D435CB` 是国内机器人生态（如睿尔曼 ALOHA、中天晨拓/华传等渠道）常见的 **Intel 原厂 D4 主板 + D430 光学模组封装版（C系列外壳）**。USB PID、寄存器协议、标定内参与标准版 **RealSense D435 (`8086:0b07`) 100% 通用**，也正因外壳组装自整机厂，外壳标签 S/N (`215322076692`) 与内部 D4 芯片电子序列号 (`218423029391`) 不同。 |
| **当前 USB 协商速率** | **SuperSpeed 5000 Mbps (USB 3.2 Gen 1)** —— 已正确工作在高速 USB 3 总线 (`Bus 002 Port 008`) ✅ |

---

## 2. 设备节点拓扑与持久化路径记录

* **接入的内核 USB 端口 (`KERNELS`)**：`2-8`（USB 3.0 Root Hub `Bus 002` 下第 8 端口）
* **唯一序列号软链接 (`by-id`)**（注意：因双接口复合设备，Interface `1.3` 的 RGB 覆盖了 `by-id` 的 `index0`，建议直接用 RealSense SDK 按序列号 `218423029391` 打开，或按下方 `by-path` 区分）：
  * `218423029391`（全球唯一，即使多台 D435 同时插也不会像单目相机那样撞号！）

### 3 个视频捕获子设备拆解（实测 V4L2 枚举）

| 功能通道 | USB 接口 | 固定物理路径 (`by-path`) | 本次动态节点 | 支持像素格式与分辨率 |
| :--- | :--- | :--- | :--- | :--- |
| **1. 硬件深度流 (Depth)** | `2-8:1.0` (`index0`) | `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:8:1.0-video-index0` | `/dev/video2` *(元数据 `/dev/video3`)* | **`Z16 ` (16-bit 毫米级深度值)**：`1280x720`, `848x480`, `848x100`, `640x480`, `640x360`, `480x270`, `424x240`, `256x144`（最高达 90fps） |
| **2. 双目红外/校准流 (IR Left/Right)** | `2-8:1.0` (`index2`) | `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:8:1.0-video-index2` | `/dev/video4` *(元数据 `/dev/video5`)* | **`GREY`** (8-bit 单目红外): 最高 `1280x720`<br>**`Y8I `** (左右双目交织 8-bit): 最高 `1280x800`<br>**`Y12I`** (左右双目交织 12-bit): `640x400`, `1280x800`<br>**`UYVY`**: 最高 `1280x720` |
| **3. RGB 彩色相机流 (Color)** | `2-8:1.3` (`index0`) | `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:8:1.3-video-index0` | `/dev/video6` *(元数据 `/dev/video7`)* | **`YUYV` (4:2:2)**：`1920x1080`, `1280x720`, `960x540`, `848x480`, `640x480`, `640x360`, `424x240`, `320x240`, `320x180` |

---

## 3. 官方 SDK、开源驱动仓库与社区资源

> **品牌迁移提示**：原 Intel RealSense 业务现已独立为 **RealSenseAI (`realsenseai`)**，GitHub 官方组织已由 `IntelRealSense` 迁移至 **`realsenseai`**。

1. **官方核心跨平台驱动 & SDK 源码（C++ / Python / Viewer 工具）**
   * **librealsense (RealSense SDK 2.0)**：<https://github.com/realsenseai/librealsense>
   * **官方文档中心**：<https://dev.realsenseai.com/>
   * **D400 系列完整数据手册 (2026 Rev.023 PDF)**：<https://www.realsenseai.com/wp-content/uploads/2026/03/RealSense-D400-Series-Datasheet-Mar-2026.pdf>
   * **官方 Python / C++ 示例代码库**（含深度对齐彩色 `align-depth2color`、点云导出、手眼标定、多相机同步）：
     * Python 示例：<https://github.com/realsenseai/librealsense/tree/master/wrappers/python/examples>
     * C++ 示例：<https://github.com/realsenseai/librealsense/tree/master/examples>
2. **ROS 2 官方功能包**
   * **realsense-ros (`ros2-master`)**：<https://github.com/realsenseai/realsense-ros>（包名 `realsense2_camera`，支持发布 `/color/image_raw`、`/depth/image_rect_raw`、`/aligned_depth_to_color/image_raw` 及 RGBD 点云 `/depth/color/points`）
3. **具身智能 / 机械臂社区集成仓库**
   * **Hugging Face LeRobot (`RealSenseCamera`)**：<https://huggingface.co/docs/lerobot/cameras>（原生支持通过序列号 `218423029391` 同时录制 RGB + Depth）
   * **SO-ARM100 D435 手腕安装支架 STL**：<https://github.com/TheRobotStudio/SO-ARM100>
   * **3D 视觉与机械臂手眼标定开源参考**：
     * `easy_handeye2` (ROS 2 手眼标定)：<https://github.com/marcoesposito1988/easy_handeye2>
     * Open3D RealSense RGB-D 实时重建教程：<https://www.open3d.org/docs/release/tutorial/sensor/realsense.html>

---

## 4. 针对您当前系统环境（Ubuntu 26.04 + Kernel 7.0）的安装与避坑指南

经检测，您当前主机环境非常新：**`Ubuntu 26.04 LTS (Resolute Raccoon)` + `Linux Kernel 7.0.0-43-generic`**，目前系统中尚未安装 `realsense-viewer` 和 `pyrealsense2`。在这套新内核环境上使用 D435CB 有以下非常关键的实操经验：

### 4.1 方案一：Python 快速开发（最省事，无需打内核补丁）
直接通过 `pip` 安装官方预编译的 `pyrealsense2`（当前最新 `v2.59.x` 已支持 Ubuntu 26.04 / Python >=3.9）并配置官方 `udev` 权限规则：
```bash
# 1. 安装 Python SDK（建议在您的 conda / uv / venv 虚拟环境中执行）
pip install pyrealsense2 numpy opencv-python

# 2. 配置 RealSense USB 权限规则（避免每次都要 sudo 才能控制激光投射器和读取内参）
sudo curl -fsSL https://raw.githubusercontent.com/realsenseai/librealsense/master/config/99-realsense-libusb.rules -o /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 4.2 方案二：编译安装完整 C++ SDK 与 `realsense-viewer` 可视化调试工具
由于您的内核是 **Linux 7.0**，而官方 `patch-realsense-ubuntu-lts-hwe.sh` 内核补丁脚本目前仅维护到 6.14 内核，**切勿强行运行内核 patch 脚本**！社区与官方工程师（MartyG）推荐在超新内核上开启 **`-DFORCE_RSUSB_BACKEND=ON`**（基于 `libusb` 用户态后端，完全绕开内核版本限制，功能与稳定性俱佳）：
```bash
sudo apt-get update
sudo apt-get install -y git cmake build-essential libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

git clone --depth 1 https://github.com/realsenseai/librealsense.git
cd librealsense
sudo ./scripts/setup_udev_rules.sh

mkdir build && cd build
cmake .. \
  -DFORCE_RSUSB_BACKEND=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=ON \
  -DBUILD_GRAPHICAL_EXAMPLES=ON
make -j$(($(nproc)-1))
sudo make install
sudo ldconfig
```
编译完成后即可直接运行 `realsense-viewer` 查看 3D 点云、调节红外激光功率，或运行 `rs-enumerate-devices -c` 导出出厂双目内外参。

---

## 5. 代码示例与机械臂操作建议

### 5.1 Python (`pyrealsense2`) 读取对齐后的 RGB + Depth 图像（推荐模板）
> **注意**：D435 的深度视场角（87°×58°）比 RGB 视场角（69°×42°）大，且两个镜头物理位置存在偏移。做视觉抓取（Grasp Pose / 像素点转 3D 坐标）时，**必须使用 `rs.align(rs.stream.color)` 将深度图对齐到彩色图坐标系**！

```python
import pyrealsense2 as rs
import numpy as np
import cv2

SERIAL = "218423029391"  # 注意使用芯片内部序列号，而非外壳贴纸号

pipeline = rs.pipeline()
config = rs.config()
config.enable_device(SERIAL)

# 机械臂常用黄金分辨率：848x480（D430双目芯片的最佳精度档位之一）或 640x480 @ 30fps
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)

# 获取深度比例尺（通常为 0.001，即 Z16 数值 1000 代表 1.0 米）
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale (m/unit):", depth_scale)

# 创建深度向 RGB 对齐器
align = rs.align(rs.stream.color)

try:
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    depth_image = np.asanyarray(depth_frame.get_data())  # uint16, 单位 mm
    color_image = np.asanyarray(color_frame.get_data())  # uint8 BGR

    # 读取画面中心点的真实三维坐标 (X, Y, Z 单位：米)
    intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
    cx, cy = 320, 240
    dist_m = depth_frame.get_distance(cx, cy)
    xyz = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], dist_m)
    print(f"中心点距离: {dist_m:.3f}m, 相机坐标系 3D 点: {xyz}")
finally:
    pipeline.stop()
```

### 5.2 在 LeRobot 中配置该深度相机
```bash
# 扫描确认 RealSense 序列号（会输出 218423029391）
lerobot-find-cameras realsense
```
在 LeRobot 配置字典中可直接通过 `serial_number_or_name: "218423029391"` 指定，无论插在哪个 USB 3.0 口都不会和另外两个单目摄像头混淆。

### 5.3 实操使用黄金建议
1. **必须插 USB 3.0+ 接口 & 注意正反插**：偶尔快速慢插 Type-C 线可能被降级识别成 USB 2.0 (`480M`)，可通过 `lsusb -t` 检查是否为 `5000M`（本次测试已是 `5000M`）。
2. **最佳深度分辨率选择**：Intel 官方白皮书指出，D435 在 **`848x480`** 分辨率下的立体匹配精度和边缘完整性优于 `640x480`（如算法需要 `640x480`，也可直接采 `848x480` 后裁剪缩放，或直接用 `640x480`）。
3. **最小盲区 (Min-Z)**：`1280x720` 下盲区约 `28cm`，降低深度分辨率至 `848x480` 或 `640x480` 时盲区可缩短至 **`~17-20cm`**（开启 `Disparity Shift` 还可进一步看清近距离约 10-12cm 物体，非常适合装在桌面/支架俯拍或远距离腕部视角）。
4. **免驱动应急退路**：即便不装 `librealsense`，您也可以直接用普通 OpenCV 打开 `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:8:1.3-video-index0` 把它当普通 1080P RGB 摄像头用；但要拿对齐深度图与相机内参，请务必走 `pyrealsense2`。
