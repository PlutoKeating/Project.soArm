# 第三方内容声明

本仓库自己的代码与文档以 MIT 许可发布，见 [LICENSE](LICENSE)。下面是随仓库或随构建产物一起分发的第三方内容，它们各自保留原许可。

## 随仓库分发

| 内容 | 来源 | 许可 | 改动 |
|---|---|---|---|
| `backend/soarm/assets/so101_new_calib.urdf` | [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) 的 `Simulation/SO101/so101_new_calib.urdf` | Apache-2.0，许可原文见 `backend/soarm/assets/LICENSE-Apache-2.0.txt` | 只保留 link / joint 的运动学定义，去掉了 visual、collision、inertial、transmission |

## 随文档站点的构建产物分发

`website/` 构建时会把下面的字体打包进 `dist/`。字体文件本身不在仓库里，由 npm 安装。

| 字体 | npm 包 | 许可 |
|---|---|---|
| Noto Sans SC | `@fontsource-variable/noto-sans-sc` | SIL Open Font License 1.1 |
| Overpass | `@fontsource-variable/overpass` | SIL Open Font License 1.1 |
| JetBrains Mono | `@fontsource-variable/jetbrains-mono` | SIL Open Font License 1.1 |

## 不随仓库分发

- Python 与 npm 依赖由 `uv` 和 `npm` 按 `pyproject.toml`、`website/package.json` 安装，各自遵循自己的许可。
- MediaPipe 手部模型由 `scripts/fetch_models.sh` 下载到 `models/`，不入库。
- `scripts/fetch_references.sh` 下载到 `third_party/` 的社区与官方源码只供对照阅读，不入库，也不属于本项目。
