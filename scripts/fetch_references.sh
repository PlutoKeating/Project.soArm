#!/usr/bin/env bash
# 下载社区与官方的参考源码到 third_party/（不入库，仅供阅读和对照）。
# 用法：scripts/fetch_references.sh
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p third_party

repos=(
  "huggingface/lerobot"                # 官方框架：电机总线、校准、SO follower、运动学
  "Vector-Wangel/XLeRobot"             # 双 SO-101 + 头部云台 + RealSense，整机形态最接近
  "TheRobotStudio/SO-ARM100"           # SO-100/101 官方硬件仓库：URDF、STL、相机支架
  "steven-tired/mediapipe-so101"       # MediaPipe 手腕位姿 -> IK 遥操作
  "MattiArlo/so101-hand-teleop"        # MediaPipe 双手遥操作 SO-100/101
  "henrynitzberg/SO101_Human_Teleop"   # 双相机三角化手腕 3D 位置
  "cal29292929/pai-so101-final"        # 腕部 eye-in-hand 相机 + 视觉伺服
  "sowmiah08/hand_based_manipulation"  # 深度相机 + MediaPipe 指尖定位
  "box2ai-robotics/lerobot-esp32"      # 键盘 IK 控制的另一种实现
)

for repo in "${repos[@]}"; do
  name="${repo##*/}"
  if [ -d "third_party/$name/.git" ]; then
    echo "已存在，更新: $name"
    git -C "third_party/$name" pull --ff-only --depth 1 || echo "  更新失败，保留现有版本"
  else
    echo "下载: $repo"
    git clone --depth 1 "https://github.com/$repo.git" "third_party/$name"
  fi
done
