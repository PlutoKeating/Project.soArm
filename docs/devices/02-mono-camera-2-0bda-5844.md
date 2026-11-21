# 单目摄像头 #2 硬件识别与使用指南

> 识别时间：2026-11-21  
> 设备角色：SO-ARM / 具身智能单目 RGB 摄像头 #2

---

## 1. 设备硬件身份（Hardware Identity）

| 属性字段 | 实测值 / 识别结果 | 与单目摄像头 #1 对比 |
| :--- | :--- | :--- |
| **USB Vendor ID (`idVendor`)** | `0x0bda` (Realtek Semiconductor Corp.) | 完全一致 |
| **USB Product ID (`idProduct`)** | `0x5844` (USB Camera) | 完全一致 |
| **固件修订版本 (`bcdDevice`)** | `12.34` (`ID_REVISION=1234`) | 完全一致 |
| **厂商字符串 (`iManufacturer`)** | `Generic` | 完全一致 |
| **产品字符串 (`iProduct`)** | `USB Camera` | 完全一致 |
| **硬件序列号 (`iSerial`)** | `200901010001` | ⚠️ **完全一致（公版写死序列号）** |
| **USB 主控芯片 (ISP)** | Realtek RTS5844 / RTS5822 系列 USB 2.0 UVC 控制器 | 同型号同批次固件 |
| **CMOS 图像传感器** | OmniVision **OV9732** (1/4", 1MP, 1280×720) | 同型号传感器 |
| **公版模组型号** | **HBVCAM-W20217 V22** / Seeed Studio RGB USB Camera-100°-720P (`SKU: 100079619`) | 同款模组 |
| **扩展单元状态 (`XU Unit 4`)** | `Selector 10 = 0008bbbbbbbbbbbb` | 完全一致 |
| **内核驱动** | Linux 原生 `uvcvideo`（UVC 1.00 免驱） | 免驱即插即用 |

### 本次接入时的端口与唯一区分标识记录
* **接入的内核 USB 端口 (`KERNELS`)**：`1-13`（接口 `1-13:1.0`，对应 udev `ID_PATH=pci-0000:00:14.0-usb-0:13:1.0`）
* **持久化物理端口路径 (`by-path`，强烈推荐用来唯一锁定 #2)**：
  * 视频捕获节点：`/dev/v4l/by-path/pci-0000:00:14.0-usb-0:13:1.0-video-index0`
  * 元数据节点：`/dev/v4l/by-path/pci-0000:00:14.0-usb-0:13:1.0-video-index1`
* **按 ID 路径 (`by-id`，⚠️ 不可用于区分 #1 和 #2)**：
  * `/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0`（两台同时接入时，后枚举的设备会覆盖此软链接）
* **动态节点（本次单独接入时为）**：`/dev/video3`（捕获）、`/dev/video4`（元数据）
  * *注：内置笔记本摄像头占用了 `/dev/video0` 和 `/dev/video1`；刚才拔插切换时原 `/dev/video2` 尚未释放或序号顺延，因此内核分配了 `/dev/video3`，这也直观证明了决不能硬编码 `/dev/videoN` 数字！*

---

## 2. 双同款单目摄像头（#1 vs #2）区分与固定方案

由于两台单目摄像头的 USB 固件描述符（含序列号 `200901010001`）100% 相同，**区分它们的唯一可靠硬件锚点是「物理 USB 拓扑路径 (`ID_PATH` / `KERNELS`)」**：

| 摄像头编号 | 实测物理接口 (`KERNELS`) | 固定不变的 `by-path` 捕获设备路径 | 建议物理标记 |
| :--- | :--- | :--- | :--- |
| **单目摄像头 #1** | `1-3:1.0` | `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0` | 标签贴纸 `CAM-1`（固定插 `1-3` 口） |
| **单目摄像头 #2** | `1-13:1.0` | `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:13:1.0-video-index0` | 标签贴纸 `CAM-2`（固定插 `1-13` 口） |

> 💡 **特别提醒**：每个 UVC 摄像头在 Linux 下都会生成**两个** `video` 节点：
> * `...-video-index0`：**真正的图像捕获节点**（`ID_V4L_CAPABILITIES=:capture:`），程序必须打开这个！
> * `...-video-index1`：UVC 元数据节点，打开后无法读取视频帧。

### 自动扫描并列出当前所有摄像头物理路径的辅助脚本
若今后更换了电脑或加装了 USB Hub，可随时运行以下单行命令查看两台 `0bda:5844` 各自对应的固定路径：
```bash
for dev in /dev/v4l/by-path/*-video-index0; do
  echo "$dev -> $(readlink -f "$dev") | $(udevadm info --query=property --name="$dev" | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_PATH=' | tr '\n' ' ')"
done
```

---

## 3. 支持的视频格式与帧率参数

与单目摄像头 #1 完全一致：
* **MJPEG (`MJPG`) — 多机并发必选**：
  * `1280x720 @ 30fps`、`960x540 @ 30fps`、`848x480 @ 30fps`、`800x600 @ 30fps`、`640x480 @ 30fps`、`352x288 @ 30fps`、`320x240 @ 30fps`、`176x144 @ 30fps`、`160x120 @ 30fps`
* **YUYV (`YUY2`) — 未压缩**：
  * `1280x720 @ 10fps`、`960x540 @ 15fps`、`848x480 @ 15fps`、`800x600 @ 15fps`、`640x480 @ 30fps` 及以下 30fps

---

## 4. 驱动源码、开源生态与参考资料

* **内核驱动**：Linux 主线 `uvcvideo` 免驱支持（`CONFIG_USB_VIDEO_CLASS`）。
* **LeRobot (Hugging Face) 相机集成文档**：<https://huggingface.co/docs/lerobot/cameras>
* **Seeed Studio SO-ARM100/101 双相机/多相机配置指南**：<https://wiki.seeedstudio.com/lerobot_so100m_new/#add-cameras>
  * *官方文档特别强调：多路摄像头采集务必在配置中加上 `fourcc: "MJPG"`，避免 `YUYV` 导致帧率暴跌与机械臂遥操作卡顿；且尽量避免将多路高带宽相机挂在同一无源 USB 2.0 Hub 下。*
* **社区虚拟机/容器穿透同序列号避坑文章**：
  * 若未来需要将两台 `0bda:5844` 同时穿透进 VirtualBox / QEMU / Docker：Docker 直接通过 `--device=/dev/v4l/by-path/...` 映射即可；若用 VirtualBox，需按 Host USB Port 过滤而非仅按 `VID:PID+Serial` 过滤（因为两台序列号都是 `200901010001`）。

---

## 5. 双单目摄像头同时采集测试代码（Python OpenCV）

当您将 #1 插回 `1-3` 口、#2 保持在 `1-13` 口时，可用此脚本同时验证两路 30fps 视频流：

```python
import cv2

CAMERAS = {
    "mono_1": "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0",
    "mono_2": "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:13:1.0-video-index0",
}

caps = {}
for name, path in CAMERAS.items():
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # 或 1280
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 或 720
    cap.set(cv2.CAP_PROP_FPS, 30)
    caps[name] = cap

for name, cap in caps.items():
    ok, frame = cap.read()
    print(f"[{name}] open={cap.isOpened()}, read={ok}, shape={frame.shape if ok else None}")
    cap.release()
```
