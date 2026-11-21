#!/usr/bin/env bash
# 借一只臂，在本 worktree 里启动开发侧控制台做真机试运行；结束时还给生产。
# 用法：scripts/trial.sh start <left|right>   确认没人互动后借臂并启动（借不到就不启动）
#       scripts/trial.sh stop                 结束试运行，恢复全量展示
#       scripts/trial.sh status
# 试运行的控制台以 --lab 方式运行：生产侧一键恢复时它会自动退出。
set -euo pipefail
cd "$(dirname "$0")/.."
PROD="${SOARM_URL:-http://127.0.0.1:8000}"
DEV_PORT="${SOARM_DEV_PORT:-8100}"

lent() {
  curl -sf -m 3 "$PROD/api/state" | python3 -c "import json,sys; print(json.load(sys.stdin)['arms']['$1'].get('lent', False))"
}

case "${1:-}" in
  start)
    arm="${2:?用法: trial.sh start <left|right>}"
    scripts/lab.sh lend "$arm" || { echo "没借到 $arm，不启动试运行"; exit 1; }
    [ "$(lent "$arm")" = "True" ] || { echo "$arm 没有处于借出状态，不启动试运行"; exit 1; }
    mkdir -p calibration/trial
    setsid nohup .venv/bin/soarm-console --lab >> calibration/trial/console.log 2>&1 &
    for _ in $(seq 1 40); do curl -sf -m 1 "http://127.0.0.1:$DEV_PORT/api/meta" >/dev/null 2>&1 && break; sleep 0.5; done
    sleep 6
    SOARM_URL="http://127.0.0.1:$DEV_PORT" scripts/lab.sh status
    ;;
  stop)
    scripts/lab.sh restore
    ;;
  status)
    echo -n "生产: "; scripts/lab.sh status
    echo -n "试运行: "; SOARM_URL="http://127.0.0.1:$DEV_PORT" scripts/lab.sh status 2>/dev/null || echo "未运行"
    ;;
  *)
    sed -n '2,6p' "$0" | sed 's/^# \{0,1\}//'; exit 2
    ;;
esac
