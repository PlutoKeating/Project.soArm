# 套机档案

每套机械臂套机一个目录，记录“跟着套机走”的参数，换一台电脑也不变：

| 文件 | 内容 |
|---|---|
| `config.yaml` | 三块控制板的串口（按 USB 序列号）、深度相机序列号、云台舵机 ID 与工作刻度 |
| `extrinsics.yaml` | 深度相机相对两只臂的外参，以及腕部相机的安装偏角 |

不在这里的是“跟着电脑走”的参数：两个腕部相机的设备路径取决于它们插在电脑的哪个 USB 口，每台电脑都要用 `soarm-tools cameras` 重新确认。

## 在新电脑上使用已有套机

```bash
cp config/kits/kit-01/config.yaml config/config.yaml
mkdir -p calibration && cp config/kits/kit-01/extrinsics.yaml calibration/
uv run soarm-tools cameras      # 把两个腕部相机的路径填进 config/config.yaml
```

外参只在下面的条件下成立，不满足就重新标定（见 `docs/QUICK_START.md` 的外参标定一节）：

- 两只臂和相机立柱的相对位置没有动过。
- 云台在工作刻度上：左右 2047，俯仰 1705（相机低头约 25°）。

## kit-01

2026-11-21 展会使用的套机。实测记录见 `docs/devices/VERIFIED.md`。

| 项目 | 值 |
|---|---|
| 左臂 / 右臂 | 各 6 个 STS3215，ID 1–6，实测供电约 5 V |
| 云台 | 2 个 STS3215，ID 7 左右、ID 8 俯仰，实测供电约 7.7 V |
| 云台刻度 | 工作姿态 俯仰 1705；手眼标定姿态 俯仰 2087（低头约 59°）；刻度减小是抬头，11.4 刻度 / 度 |
| 中柱避让 | 左臂肩部旋转不超过向右 30°，右臂不超过向左 30°（代码默认值） |
| 腕部相机安装偏角 | 左臂偏右 4.7°、偏下 14.9°；右臂样本不足，暂用偏右 3.5°、偏下 19.6° |

## 新增一套套机

从 `config/config.example.yaml` 开始走完 `docs/QUICK_START.md` 的首次上电流程，跑通后把得到的 `config/config.yaml`（去掉腕部相机路径）和 `calibration/extrinsics.yaml` 存成 `config/kits/kit-02/`。
