# SO-ARM 摄像头硬件识别总览与三机协同配置指南

> **阅读前注意**：本目录的三篇设备报告是在最初那台电脑上生成的。其中 `/dev/v4l/by-path/pci-…` 路径、`/dev/videoN` 编号和 USB 口位都**跟着电脑走**，换一台电脑就不同，只能当作示例。跟着套机走的参数（控制板序列号、深度相机序列号、云台舵机 ID 与刻度、外参）以 [`config/kits/`](../../config/kits/README.md) 和 [VERIFIED.md](VERIFIED.md) 为准。

> **注意**：本报告有几处与后来的实测不一致（USB 口位、路径写法、RealSense 序列号），见 [VERIFIED.md](./VERIFIED.md)，冲突时以它为准。

> 生成日期：2026-11-21  
> 主机环境：Ubuntu 26.04 LTS (`Linux 7.0.0-43-generic x86_64`)  
> 文档目录：`~/soArm-devices-detect/`

---

## 1. 全部设备清单与核心身份速查表

| 编号 | 设备类型 | 识别型号 | `VID:PID` | 内部序列号 (`iSerial`) | 实测插口 (`KERNELS`) | 推荐软件绑定方式 | 详细文档 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **#1** | 单目摄像头 1 | **OV9732 720P UVC 模组**<br>*(Realtek RTS5844/5822, HBVCAM / Seeed 100079619)* | `0bda:5844` | `200901010001`<br>*(⚠️与 #2 撞号)* | `1-3` *(USB 2.0 480M)* | **必须用物理端口路径 `by-path`**：<br>`/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0` | [01-mono-camera-1-0bda-5844.md](./01-mono-camera-1-0bda-5844.md) |
| **#2** | 单目摄像头 2 | **OV9732 720P UVC 模组**<br>*(Realtek RTS5844/5822, HBVCAM / Seeed 100079619)* | `0bda:5844` | `200901010001`<br>*(⚠️与 #1 撞号)* | `1-13` *(USB 2.0 480M)* | **必须用物理端口路径 `by-path`**：<br>`/dev/v4l/by-path/pci-0000:00:14.0-usb-0:13:1.0-video-index0` | [02-mono-camera-2-0bda-5844.md](./02-mono-camera-2-0bda-5844.md) |
| **#3** | 双目深度相机 | **Intel RealSense D435 (`D435CB`)**<br>*(外壳标签 S/N: `215322076692`)* | `8086:0b07` | **`218423029391`**<br>*(✅全球唯一)* | `2-8` *(USB 3.2 5000M)* | **用内部芯片序列号**：`218423029391`<br>*(纯 V4L2 RGB 路径：`pci-0000:00:14.0-usb-0:8:1.3-video-index0`)* | [03-depth-camera-realsense-d435cb.md](./03-depth-camera-realsense-d435cb.md) |

> *(注：主机自带内置笔记本摄像头为 `04f2:b7b8` Chicony/SunplusIT，固定在 `usb-0:11:1.0`，默认占用 `/dev/video0` 和 `/dev/video1`，采集机械臂数据时请注意避开索引 `0`。)*

---

## 2. 三大核心发现（必读避坑总结）

1. **两个单目摄像头硬件 ID 完全相同，切勿用 `/dev/videoN` 或 `by-id`**：
   * 两颗 OV9732 单目相机的 `VID:PID` (`0bda:5844`)、版本号 (`12.34`)、序列号 (`200901010001`) 100% 一致。
   * **最佳实践**：将单目 #1 固定插在刚才测试第 1 个的 USB 口 (`1-3`)，单目 #2 固定插在刚才测试第 2 个的 USB 口 (`1-13`)，并用贴纸标记。代码里统一使用 `by-path` 的 `...-video-index0` 路径（或者部署下方的一键 `udev` 规则生成 `/dev/cam_mono_1` 与 `/dev/cam_mono_2`）。
2. **深度摄像头 `D435CB` 的外壳 S/N 与芯片内部 S/N 不同**：
   * 外壳贴纸印的是整机渠道组装号 `215322076692`，而内部 Intel D4 视觉芯片枚举出来的真实电子序列号为 **`218423029391`**。在 `pyrealsense2`、`realsense-ros` 和 `LeRobot` 中填写序列号时，**必须填写 `218423029391`**。
3. **USB 总线天然物理隔离，三机同时满帧运行无压力**：
   * 从实测拓扑看，深度相机走的是 **`Bus 002`（20Gbps xHCI USB 3.x 高速总线，协商在 5000Mbps）**，而两颗单目相机走的是 **`Bus 001`（480Mbps USB 2.0 总线）**。
   * 只要两颗单目相机在代码中显式指定 **`MJPG`** 压缩格式（`fourcc="MJPG"`），三台相机同时开启 `30fps` 采集不会发生任何 USB 带宽争抢！

---

## 3. 一键固定设备别名的 `udev` 规则（可选推荐）

如果您希望在代码中使用直观的 `/dev/cam_mono_1` 和 `/dev/cam_mono_2`，并赋予 RealSense 免 `sudo` 权限，可将以下内容保存至 `/etc/udev/rules.d/99-soarm-cameras.rules`：

```udev
# 1. 单目摄像头 #1 (固定于 USB 端口 1-3) -> /dev/cam_mono_1
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="5844", KERNELS=="1-3:1.0", ATTR{index}=="0", SYMLINK+="cam_mono_1"

# 2. 单目摄像头 #2 (固定于 USB 端口 1-13) -> /dev/cam_mono_2
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="5844", KERNELS=="1-13:1.0", ATTR{index}=="0", SYMLINK+="cam_mono_2"

# 3. RealSense D435CB (S/N: 218423029391) 用户态 USB 访问权限 + RGB 快捷软链接 -> /dev/cam_d435_rgb
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE:="0666", GROUP:="plugdev"
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", ATTRS{serial}=="218423029391", KERNELS=="*:1.3", ATTR{index}=="0", SYMLINK+="cam_d435_rgb"
```

启用命令：
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## 4. 三台相机在 LeRobot 中联合调用配置模板

```yaml
cameras:
  wrist_left:
    type: opencv
    index_or_path: "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0"
    width: 640
    height: 480
    fps: 30
    fourcc: "MJPG"

  wrist_right:
    type: opencv
    index_or_path: "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:13:1.0-video-index0"
    width: 640
    height: 480
    fps: 30
    fourcc: "MJPG"

  overhead_rgbd:
    type: realsense
    serial_number_or_name: "218423029391"
    width: 640
    height: 480
    fps: 30
    use_depth: true
```
