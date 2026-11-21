# 快速开始

## 1. 安装

需要 Linux、[uv](https://docs.astral.sh/uv/) 和 Python 3.12（uv 会自动下载）。

```bash
uv sync --extra realsense     # 含 LeRobot、MediaPipe、pyrealsense2；首次安装会下载 torch，体积较大
scripts/fetch_models.sh       # 下载 MediaPipe 手部模型到 models/
uv run pytest                 # 可选：跑一遍不依赖硬件的测试
```

串口权限：当前用户需要在 `dialout` 组里（`groups` 查看；不在则 `sudo usermod -aG dialout $USER` 后重新登录）。

可选：`scripts/fetch_references.sh` 把调研过的社区与官方源码下载到 `third_party/` 供对照阅读，约 1.2 GB。

## 2. 不接硬件先跑通

```bash
uv run soarm-console --mock
```

打开 <http://127.0.0.1:8000>。模拟模式下机械臂和云台是软件替身，三种模式的逻辑都可以操作；相机若已在配置里填写则使用真实相机。

## 3. 真机首次上电检查表

以下步骤每台设备只需做一次，结果写入 `config/config.yaml`（从 `config/config.example.yaml` 复制，不入库）。

### 3.1 接线与供电

- 三块控制板的跳线帽都在 **B 位（USB ↔ SERVO）**。
- 每块控制板的 DC 口接舵机电源，只插 USB 时舵机不应答。STS3215 有 7.4 V 和 12 V 两种版本，**上电前务必核对舵机标签上的电压与电源一致**，接错会烧舵机。
- 上电时手臂会保持无力状态，先把它摆在自然收拢的位置。

### 3.2 找到串口并分辨三块板

```bash
uv run soarm-tools scan
```

输出每个串口上的舵机 ID、电压、位置和已保存的限位。这一步只读。ID 为 1–6 的是机械臂，只有 2 个舵机的是云台。把路径填入 `arms.left.port`、`arms.right.port`、`head.port`，把云台的两个 ID 填入 `head.pan.id`、`head.tilt.id`。左右臂分不清时，运行 `soarm-tools watch left`，用手动一下某只臂，看读数是否变化。

### 3.3 核对机械臂校准

控制台直接使用舵机里已保存的校准（零点偏置和行程限位），不需要校准文件。`scan` 输出里各关节的“限位”不是 `[0, 4095]`（腕部旋转除外）就说明已校准过。

用下面的命令核对。它是只读的，不改变力矩状态；力矩关闭时可以用手摆动手臂：

```bash
uv run soarm-tools watch left
```

它会先列出这只臂的行程限位，再实时显示角度。收拢停放时，肩部俯仰应在 −100° 左右、肘部在 90° 左右；把手臂水平向前伸直时，前 5 个关节应都接近 0°。偏差超过 10° 才需要重新校准（用手把每个关节掰过完整行程，结果写入舵机）：

```bash
uv run soarm-tools calibrate left
```

本套设备两只臂的校准已核对可用，见 [devices/VERIFIED.md](devices/VERIFIED.md)。

### 3.4 云台

云台的两个舵机出厂时没有校准，而且停放位置可能紧贴编码器的 0/4095 回绕点，直接用会在转过回绕点时绕一整圈。所以先把工作姿态设成刻度中点：

1. `uv run soarm-tools scan` 找到云台串口和两个舵机的 ID，填入 `head.port`、`head.pan.id`、`head.tilt.id`。分不清哪个是俯仰时，运行 `uv run soarm-tools head-watch`，用手让相机点头，看哪个读数在变。
2. 用手把相机摆到工作姿态并扶住：朝正前方，画面要覆盖两臂前方观众伸手的区域（台前 0.3–1 米、腰到胸口高度）。**不要让它朝下看地面**，否则深度相机和腕部相机看到的不是同一只手，后面的标定和引导都无从谈起。
3. 把当前姿态设为零点并上力矩（LeRobot 的 `set_half_turn_homings`）。之后 `center` 就是 2047，`min` / `max` 在它两侧留出需要的范围。

本套设备的云台参数见 [devices/VERIFIED.md](devices/VERIFIED.md)。

### 3.5 相机

```bash
uv run soarm-tools cameras
```

每个相机会存一张快照，据此分辨左腕和右腕，把路径填入 `cameras.wrist_left.device`、`cameras.wrist_right.device`。注意三点：

- 两个腕部相机型号和序列号完全相同，只能按 USB 口区分，换插到别的口后需要重新填写。
- 路径必须用带 `usbv2` 的写法；不带版本号的 by-path 在 USB2 与 USB3 的同号口上会重名。
- 快照分不清左右时，启动控制台后把手依次放到两只臂的夹爪前，看手出现在哪个面板。“左”按机器人自己的朝向算。

画面是倒的就设 `rotate: 180`。RealSense 的序列号以该命令输出的为准，填入 `cameras.head.device`（只有一台时可留空）。

本套设备的实测结果见 [devices/VERIFIED.md](devices/VERIFIED.md)。

### 3.6 启动并逐项验证

```bash
uv run soarm-console
```

建议顺序，每一步都把手放在空格键上。首次上电建议一次只动一只臂：在另一只臂的面板上点“停用”，它就只保持当前位置。

1. 页面上两臂都显示已连接，三路画面正常。连接时手臂会上力矩并停在原地。
2. 按 `G` 回初始位。姿态应是夹爪水平朝前、离基座约 26 cm，腕部相机画面是正的。画面是侧的就调整 `home.wrist_roll`（本套设备是 −90°）。
3. 按 `1` 进入键盘控制，逐个轻点方向键，确认前后、左右、上下方向与界面标注一致。
4. 按 `2` 进入腕部跟随。把手伸到腕部相机前 30–40 cm 并慢慢向各方向移动：
   - 手臂应朝手的方向转。如果反而躲开，把该臂的 `follow_pan_sign` 或 `follow_tilt_sign` 改为 `-1`。
   - 朝中柱方向转到极限时应留有安全距离，不够就收紧 `arms.<名字>.joint_limits.shoulder_pan`。
   - 手拿开约 1.5 秒后应回初始位。
5. 点“收到休息位并释放力矩”。休息位就是连接时手臂停放的姿态。

### 3.7 外参标定（深度引导需要）

前提：云台已固定在工作姿态，之后不要再动；深度相机画面里能看到观众伸手的区域。

**自动方式（推荐）。** 进入腕部跟随或深度引导模式，用一只手在每只臂前方慢慢移动，覆盖远近、高低、左右。腕部相机和深度相机同时看到这只手时会自动积累样本，页面“外参标定”里显示样本数。每只臂攒够 30 对以上后点“用自动样本求解所选臂”。采样期间深度相机画面里只能有一只手。

**旁路方式（不想打断正在运行的控制台时）。** 运行中的控制台把深度相机设为 `enabled: false`，另开一份代码（例如 git worktree）执行：

```bash
uv run soarm-tools collect-calib --prod http://127.0.0.1:8000 --seconds 600
uv run soarm-tools solve-calib
```

它自己读深度相机，只读轮询运行中控制台的状态接口，结果写入 `calibration/extrinsics.yaml`。

**手动方式。** 在页面“外参标定”里打开“采点”，每只臂在不同位置把夹爪尖移到深度相机看得见的地方，在深度画面上点击夹爪尖，8 个点左右后求解。

判断标定好坏：求解给出的偏差在几厘米以内，并且算出的相机位置与实物相符（应在两臂之间的上方）。偏差有几十厘米说明样本里混进了别的手，清空重采。

之后按 `3` 进入深度引导：把手伸到深度相机看得见、腕部相机看不见的位置，较近的那只臂应转过去对准手。

## 4. 日常使用

调试时直接在终端运行，Ctrl+C 结束（会自动先收到休息位再释放力矩）：

```bash
uv run soarm-console                 # 启动后停在待机
uv run soarm-console --mode follow   # 启动后自动进入腕部跟随
```

## 5. 生产运行（展示时）

用 systemd 用户服务常驻运行：脱离终端，异常退出会自动拉起并回到 `start_mode` 指定的模式。

```bash
scripts/production.sh start      # 启动
scripts/production.sh status     # 查看状态
scripts/production.sh logs       # 跟随查看日志
scripts/production.sh stop       # 停止：两只臂先收到休息位再释放力矩
```

准备工作：

- 在 `config/config.yaml` 里设 `start_mode: follow`。
- 生产运行以仓库目录里当前检出的代码为准，应检出 `main` 分支。
- 不需要的相机设 `enabled: false`，设备就不会被占用。
- 运行期间不要在这个目录里切换分支或改代码；后续开发在单独的 git worktree 里做，并使用别的端口。

异常重启时手臂的力矩不会掉：舵机保持在原位，新进程接管后回到初始位继续工作。

### 展示与试验开发之间的快速切换

展会现场随时会来人。试验新功能需要真机时，不重启生产，而是临时“借走”一只臂；来人了几秒内还回去。

```bash
scripts/lab.sh status          # 当前状态
scripts/lab.sh lend right      # 确认没人在互动后，把右臂借给开发程序；左臂照常展示
scripts/lab.sh restore         # 恢复全量展示（也可以点网页顶部的“恢复全量展示”）
scripts/lab.sh deploy          # 没人互动时重启生产，加载主目录当前检出的代码
```

- **借出**：生产停止指挥这只臂并松开串口，舵机力矩不变，臂停在原位。开发程序（在另一个 worktree 里）随后可以打开这个串口。
- **恢复**：生产先让登记在案的开发程序退出，再重新接管借出的臂并回到当前模式。开发程序启动时要调用 `soarm.lab.claim()` 登记，并在收到终止信号后立刻松开串口、不动力矩。
- **空闲判断**：`lend` 和 `deploy` 默认先等所有在展示的臂连续 8 秒没在跟手；120 秒内一直有人就放弃。加 `--force` 跳过。
- 借出期间网页顶部有黄色提示条，写明哪只臂没有参与展示。

## 6. 常见问题

| 现象 | 处理 |
|---|---|
| `scan` 没有舵机应答 | 检查 DC 电源和跳线帽位置；换一根 USB 线 |
| 页面显示某关节“行程限位是出厂值” | 该臂没有校准过，按 3.3 运行 `soarm-tools calibrate` |
| 手臂动到一半提示“受到阻挡，已让位” | 有东西挡住了，或速度相对负载过快。排除遮挡后重试；确属误报可调大 `safety.stall_error_deg` |
| 键盘控制提示“到达可达范围边界” | 目标超出了机构可达范围或工作空间包围盒，往回移动即可 |
| 深度引导不动，面板提示云台偏离标定姿态 | 点“云台回到标定姿态” |
| 跟随发抖或来回摆 | 调小 `follow.gain` 或 `follow.lead_s`；冲过头再回来就调小 `follow.aim_scale` |
| 跟随跟不上 | 先看页面上腕部相机的帧率是否约 30；再调大 `follow.gain`、`follow.max_speed_deg_s` |
| RealSense 打不开 | 确认接在 USB 3 口；`uv run soarm-tools cameras` 查看是否被识别；序列号填的是否是该命令输出的那个 |
