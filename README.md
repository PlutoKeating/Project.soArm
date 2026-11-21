<div align="center">

# SO-ARM 双臂演示控制台

**观众伸手，机械臂转过去看着手，并跟着动。**

一套双 SO-100/101 机械臂演示套机的浏览器控制台：键盘控制、腕部相机手部跟随、深度相机全景引导，以及路演现场需要的全部安全与值守工具。

[快速上手](#快速上手) · [四种模式](#四种模式) · [路演值守](#路演值守) · [文档](#文档) · [当前状态](#当前状态)

</div>

> [!IMPORTANT]
> 这是会动的真机。任何时候按 **空格** 急停。首次上电请按 [docs/QUICK_START.md](docs/QUICK_START.md) 一步步来，每一步都把手放在空格键上。

## 概览

这个仓库是一次展会的完整交付物：代码、配置模板、套机档案和当天的实测记录。它面向两类人：

- **接手路演的同事**：拿着现成的套机和一台新电脑，照着文档把演示跑起来并在现场值守。
- **新加入的同学**：把它当作上手真机的第一个项目，在它上面做试验。

套机的组成是固定的：

| 部件 | 型号 | 数量 |
|---|---|---|
| 机械臂 | SO-100 / SO-101 follower，每只 6 个 Feetech STS3215 | 2 |
| 腕部相机 | OV9732 720P UVC 模组，装在夹爪上方 | 2 |
| 深度相机 | Intel RealSense D435，装在 2 自由度云台上 | 1 |
| 云台 | 2 个 Feetech STS3215 | 1 |
| 控制板 | Waveshare Bus Servo Adapter (A)，左臂、右臂、云台各一块 | 3 |

上位机是一台 Linux 电脑，所有操作在浏览器里完成。

## 快速上手

需要 Linux、[uv](https://docs.astral.sh/uv/)（Python 3.12 由它自动下载）。

### 1. 不接硬件，先看到控制台

```bash
uv sync --extra realsense        # 安装依赖；首次会下载 torch，体积较大
scripts/fetch_models.sh          # 下载 MediaPipe 手部模型
uv run soarm-console --mock      # 机械臂和云台用软件替身
```

打开 <http://127.0.0.1:8000>，四种模式都可以在模拟臂上操作。

### 2. 接上现有的套机

套机自带的参数（控制板串口、深度相机序列号、外参）已经记录在 [config/kits/](config/kits/) 里，换一台电脑不变：

```bash
cp config/kits/kit-01/config.yaml config/config.yaml
mkdir -p calibration && cp config/kits/kit-01/extrinsics.yaml calibration/
uv run soarm-tools cameras       # 找到两个腕部相机的路径，填进 config/config.yaml
```

只有两个腕部相机的路径要重新填：它们的序列号相同，只能按插在电脑哪个 USB 口来区分。

然后把云台转到工作姿态，启动：

```bash
uv run soarm-tools scan          # 确认三块控制板都有应答；只有 2 个舵机的那个串口是云台
uv run soarm-tools head-set --port <云台串口> --pan-id 7 --tilt-id 8 --pan 2047 --tilt 1705
uv run soarm-console
```

> [!WARNING]
> 上电前核对舵机标签上的电压与电源一致。这套设备的舵机都是 7.4 V 版（耐压上限 8 V），实测机械臂供电约 5 V、云台约 7.7 V；接 12 V 电源会烧舵机。

### 3. 全新的套机

同型号的新套机要走一遍首次上电流程：分辨三块控制板、核对校准、给云台设零点、标定外参。从 [config/config.example.yaml](config/config.example.yaml) 开始，照着 [docs/QUICK_START.md](docs/QUICK_START.md) 做。

> [!TIP]
> 仓库里还有一个图文并茂的上手站点，内容与本文档一致、步骤更细：`cd website && npm install && npm run dev`。

## 四种模式

| 模式 | 快捷键 | 做什么 |
|---|---|---|
| 待机 | `0` | 保持当前位置 |
| 键盘控制 | `1` | 末端笛卡尔控制。左右两套键位同时控制两只臂，按住才动，松开即停 |
| 腕部跟随 | `2` | 腕部相机看到手就转过去对准它，只转动、不调距离；看不到手 1.5 秒后回初始位。两只臂各跟各的 |
| 深度引导 | `3` | 深度相机在大视野里找到手的 3D 位置，把合适的那只臂转过去；手进入腕部视野后由腕部相机接管 |

其他按键：`空格` 急停，`G` 回初始位。每只臂可以在页面上单独停用。

面向公开场合，默认带这些保护：

- **限速与限位**：关节限速；限位取 URDF、舵机行程、配置三者的交集。
- **中柱避让**：两只臂朝中间转时不超过 30°，不会碰到相机立柱。
- **受阻让位**：实际位置与指令偏差过大并持续一小段时间，就放弃目标并暂停。
- **按键心跳**：浏览器失去焦点或断开，视为全部松开。
- **退出先收手**：正常退出会先收到休息位再释放力矩。

## 路演值守

展示时用 systemd 用户服务常驻运行，异常退出会自动拉起并回到配置的模式：

```bash
scripts/production.sh start      # 启动；stop / status / logs 同理
```

现场最常用的三件事：

```bash
scripts/lab.sh restore           # 来人了：几秒内恢复全量展示
scripts/lab.sh lend right        # 没人时：借出一只臂做试验，另一只照常展示
uv run soarm-tools watchdog      # 人要离开：异常时收手、释放力矩并急停
```

`config/config.yaml` 里的 `start_mode` 决定启动后进入哪种模式，展示一般用 `fused`（没有外参时用 `follow`）。完整流程见 [docs/QUICK_START.md](docs/QUICK_START.md) 的“生产运行”一节。

## 仓库结构

```text
backend/soarm/     后端：控制循环、运动学、手部检测、硬件驱动、Web 服务、命令行工具
backend/tests/     不依赖硬件的测试（模拟臂 + 假相机）
frontend/          控制台前端：单页，无构建步骤，由后端直接提供
website/           上手文档站点（Astro，部署到 Cloudflare Workers）
config/            配置模板与套机档案；本机的 config.yaml 不入库
scripts/           生产运行、借臂与恢复、下载模型
docs/              架构、接口、上手流程、设备实测记录
PRODUCT.md         产品事实：读者是谁、做什么、不能虚构什么（设计与文档工作的依据）
DESIGN.md          文档站点的设计系统（不适用于 frontend/ 的控制台）
calibration/       运行时生成的外参与样本（不入库）
models/            手部模型（脚本下载，不入库）
```

配置分三层，换电脑或换套机时只动该动的那一层：

| 跟着谁走 | 内容 | 放在哪 |
|---|---|---|
| 这类套机 | 限速、跟随参数、中柱避让、初始位 | `backend/soarm/config.py` 的默认值 |
| 这一套套机 | 控制板串口、深度相机序列号、云台刻度、外参 | `config/kits/<套机>/` |
| 这台电脑 | 腕部相机的 USB 口路径 | `config/config.yaml`（不入库） |

## 文档

| 文档 | 内容 |
|---|---|
| [docs/QUICK_START.md](docs/QUICK_START.md) | 安装、首次上电检查表、外参标定、生产运行、常见问题 |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 整体架构与每个关键设计决定的理由 |
| [docs/API.md](docs/API.md) | HTTP / WebSocket 接口 |
| [docs/devices/VERIFIED.md](docs/devices/VERIFIED.md) | 展会当天的全部真机实测记录 |
| [config/kits/README.md](config/kits/README.md) | 套机档案：哪些参数跟着套机走 |
| [docs/REFERENCES.md](docs/REFERENCES.md) | 调研过的社区与官方方案 |
| [backend/docs/](backend/docs/README.md) · [frontend/docs/](frontend/docs/README.md) · [website/docs/](website/docs/README.md) | 各模块文档 |

开发时跑一遍不依赖硬件的测试：

```bash
uv run pytest
```

## 当前状态

2026-11-21 在展会上连续运行了一整天。已在真机上验证：

- 两只臂连接不跳变、展开到初始位、退出自动收手。
- 腕部跟随：方向、丢失后回位、中柱避让、两只臂各跟各的。
- 深度引导：两只臂的引导命中都把手带进了腕部相机视野。
- 借臂与恢复、无人值守看门狗（当天未触发过真实异常，做过演练）。

> [!NOTE]
> 下面这些**还没有**在真机上验证过，使用时请当作未知：
>
> - 键盘笛卡尔控制的各个方向（只操作过腕部旋转）。
> - 伸出式引导（`fused.reach: true`）。
> - `soarm-tools head-set` 的大幅转动和 `soarm-tools tilt-calib`：两条命令是收尾时补的，前者只在原地做过定位测试，后者只有单元测试。
> - 右臂腕部相机的安装偏角样本不足，暂用左臂的估计值。
> - 新旧跟随控制器的对比只做了左右各一套的单轮，没有对调复测。

手臂伸出时有约 2–3 cm 的重力下垂，逆解没有补偿它。细节见 [docs/devices/VERIFIED.md](docs/devices/VERIFIED.md)。

## 署名与许可

本项目由 **PlutoKeating** 开发于 2026 年 11 月 21 日。

以 [MIT](LICENSE) 许可开源。随仓库分发的第三方内容见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
