#!/usr/bin/env bash
# 下载 MediaPipe 手部关键点模型到 models/（不入库）。
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p models
url="https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"
out="models/hand_landmarker.task"
if [ -s "$out" ]; then
  echo "已存在: $out"
else
  curl -fL --retry 3 -o "$out" "$url"
  echo "已下载: $out"
fi
