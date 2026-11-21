# 单目摄像头 #1 硬件识别与使用指南

> 识别时间：2026-11-21  
> 设备角色：SO-ARM / 具身智能单目 RGB 摄像头 #1

---

## 1. 设备硬件身份（Hardware Identity）

| 属性字段 | 实测值 / 识别结果 |
| :--- | :--- |
| **USB Vendor ID (`idVendor`)** | `0x0bda` (Realtek Semiconductor Corp.) |
| **USB Product ID (`idProduct`)** | `0x5844` (USB Camera) |
| **固件修订版本 (`bcdDevice`)** | `12.34` (`ID_REVISION=1234`) |
| **厂商字符串 (`iManufacturer`)** | `Generic` |
| **产品字符串 (`iProduct`)** | `USB Camera` |
| **硬件序列号 (`iSerial`)** | `200901010001` ⚠️ *(公版固件写死，与单目摄像头 #2 完全相同)* |
| **USB 主控芯片 (ISP)** | Realtek RTS5844 / RTS5822 系列 USB 2.0 UVC Camera Controller |
| **CMOS 图像传感器** | OmniVision **OV9732** (1/4", 1MP, 1280×720, 3µm×3µm 像素) |
| **公版模组型号** | **HBVCAM-W20217 V22** / Seeed Studio RGB USB Camera-100°-720P (`SKU: 100079619`) / Seengreat OV9732 U1M |
| **结构与光学特征** | 38×38mm（可折边至 32×32mm，带 `IR` 灯板开槽），M12 手动调焦广角镜头 |
| **USB 协商速率** | USB 2.0 High-Speed (480 Mbps)，最大功耗 `500mA` |
| **内核驱动** | Linux 原生 `uvcvideo`（UVC 1.00 免驱） |

### 本次接入时的端口与持久化路径记录
* **接入的内核 USB 端口 (`KERNELS`)**：`1-3`（接口 `1-3:1.0`）
* **持久化物理端口路径 (`by-path`，推荐使用)**：
  * 视频捕获节点：`/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0`
  * 元数据节点：`/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index1`
* **按 ID 路径 (`by-id`，⚠️ 双机同插时会冲突覆盖，请勿用于多机区分)**：
  * `/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0`
* **动态节点（随拔插顺序变化，本次为）**：`/dev/video2`（捕获）、`/dev/video3`（元数据）

---

## 2. 如何与「单目摄像头 #2」精准区分（重点！）

经实测对比，**单目摄像头 #1 与 #2 的芯片 EEPROM 内烧录了完全一致的描述符**（`VID:PID=0bda:5844`、`bcdDevice=12.34`、`iSerial=200901010001`）。因此软件层面无法通过 `by-id` 或序列号区分它们，**必须绑定物理 USB 接口（`by-path` / `KERNELS`）**：

### 方法 A：直接在代码或配置中使用 `/dev/v4l/by-path/`（零配置，最简单）
只要将**摄像头 #1 固定插在本次测试的 USB 口（`usb-0:3`）**，无论电脑重启还是先插谁后插谁，它的视频流永远是：
```text
/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0
```
> 注：若您之后通过外接 USB Hub 连接，接入后运行 `ls -l /dev/v4l/by-path/` 即可看到带 Hub 级联号的新固定路径（如 `...-usb-0:3.1:1.0-video-index0`），同样永久固定。

### 方法 B：配置 `udev` 规则生成语义化固定别名（如 `/dev/cam_mono_1`）
创建 `/etc/udev/rules.d/99-soarm-cameras.rules`：
```udev
# 单目摄像头 #1（绑定在 USB 端口 1-3，仅匹配 index==0 的真实视频流节点）
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="5844", KERNELS=="1-3:1.0", ATTR{index}=="0", SYMLINK+="cam_mono_1"

# 单目摄像头 #2（绑定在 USB 端口 1-13，仅匹配 index==0 的真实视频流节点）
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="5844", KERNELS=="1-13:1.0", ATTR{index}=="0", SYMLINK+="cam_mono_2"
```
生效命令：
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```
建议同时在摄像头 #1 的 USB 插头端贴上物理标签（如 `CAM1 -> 左上USB口 (1-3)`）。

---

## 3. 支持的视频格式、分辨率与控制项

### 3.1 视频流格式（实测 USB 描述符）
| 像素格式 | 分辨率列表 | 帧率 (FPS) | 带宽表现与建议 |
| :--- | :--- | :--- | :--- |
| **MJPEG (`MJPG`)** | `1280x720`, `960x540`, `848x480`, `800x600`, `640x480`, `352x288`, `320x240`, `176x144`, `160x120` | **全部支持 30 FPS** | **强烈推荐**：片上硬件 JPEG 编码，多相机并发不占满 USB 2.0 带宽 |
| **YUYV (`YUY2`)** | `1280x720` | **10 FPS** | 未压缩 16bpp，带宽占用极大，高分低帧 |
| **YUYV (`YUY2`)** | `960x540`, `848x480`, `800x600` | **15 FPS** | 不建议多相机同开时使用 |
| **YUYV (`YUY2`)** | `640x480`, `352x288`, `320x240`, `176x144`, `160x120` | **30 FPS** | 低延迟无损，单机 640×480 可用，双机同 Hub 易超带宽 |

### 3.2 UVC 硬件可调参数
* **终端控制 (Camera Terminal)**：Auto-Exposure Mode（自动/手动曝光切换）、Auto-Exposure Priority、Exposure Time (Absolute)（绝对曝光时间，防机械臂运动拖影关键参数）、Roll (Absolute)。
* **图像单元 (Processing Unit)**：Brightness、Contrast、Hue、Saturation、Sharpness、Gamma、White Balance Temperature（自动/手动色温）、Backlight Compensation、Power Line Frequency（50Hz/60Hz 防工频闪烁）。
* **Realtek 扩展单元 (XU)**：`guid={1229a78c-47b4-4094-b0ce-db07386fb938}`（Realtek 内部 ISP 寄存器访问接口，常规使用无需操作）。

---

## 4. 驱动源码、社区生态仓库与技术资料

1. **Linux 官方内核驱动源码**
   * 标准 UVC 驱动 `uvcvideo`：`drivers/media/usb/uvc/`（无需额外安装任何闭源驱动）。
2. **机械臂 / 具身智能官方与分销商文档**
   * **Hugging Face LeRobot 相机文档**：<https://huggingface.co/docs/lerobot/cameras>
   * **Seeed Studio SO-ARM100/101 相机接入教程**：<https://wiki.seeedstudio.com/lerobot_so100m_new/#add-cameras>
   * **Seeed Studio OV9732 产品页与规格书**：<https://www.seeedstudio.com/Camera-Module-OV9732-1MP-100-USB2-0-USB2-0-L38-W38-H16-05mm-p-6982.html>
   * **Seengreat OV9732 Wiki 手册**：<https://seengreat.com/wiki/135/usb-camera-1mp-72-100-ov9732-image-sensor>
   * **SO-ARM100 官方 3D 打印支架仓库**：<https://github.com/TheRobotStudio/SO-ARM100>
3. **ROS 2 / 开源驱动仓库**
   * **ROS 2 `usb_cam`**：<https://github.com/ros-drivers/usb_cam>
   * **ROS 2 `v4l2_camera`**：<https://github.com/klintan/ros2_v4l2_camera>
   * **跨平台用户态 UVC 库 `libuvc`**：<https://github.com/libuvc/libuvc>
   * **多相机按硬件路径绑定参考项目**：<https://github.com/boredhero/plant-backend>

---

## 5. 快速上手代码与实操建议

### 5.1 Python (OpenCV) 指定物理路径 + 强制 MJPG 720P@30fps 读取
```python
import cv2

# 使用 by-path 固定绑定摄像头 #1（插在 usb-0:3 端口时）
CAM1_PATH = "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0"

cap = cv2.VideoCapture(CAM1_PATH, cv2.CAP_V4L2)
# 关键：必须先设置 MJPG，否则默认 YUYV 在 720P 下只有 10fps 且占满 USB2.0 带宽
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)

ret, frame = cap.read()
if ret:
    print("成功捕获帧:", frame.shape)
cap.release()
```

### 5.2 LeRobot 命令行调用示例（使用 `by-path` 区分双单目相机）
在 LeRobot 中，`index_or_path` 支持直接传入 Linux 路径字符串，从而完美规避双相机序号漂移：
```bash
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.cameras='{
    "wrist": {"type": "opencv", "index_or_path": "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"},
    "top":   {"type": "opencv", "index_or_path": "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:13:1.0-video-index0", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}
  }'
```

### 5.3 硬件与排障注意事项
1. **带宽分配**：两个 `0bda:5844` 摄像头如果接到同一个廉价无源 USB 2.0 Hub 上，务必使用 `MJPG` 格式（`640x480` 或 `1280x720`）；若使用 `YUYV` 会触发 `VIDIOC_STREAMON: No space left on device`。尽量分别插在主机不同的 USB 口或带供电的 USB 3.0 Hub 上。
2. **避免运动模糊与光照闪烁**：室内日光灯下建议通过 `v4l2-ctl -d <设备> -c power_line_frequency=1` 设为 50Hz；若机械臂快速移动时图像糊，可关闭自动曝光并固定短曝光时间。
3. **手动对焦锁定**：M12 齿轮镜头圈可旋转对焦（手腕相机建议对焦在夹爪指尖处约 10~20cm），调准后可用少量热熔胶或生料带增加阻尼防止机械臂抖动跑焦。
