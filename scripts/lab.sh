#!/usr/bin/env bash
# 展会现场“展示”与“试验开发”之间的快速切换。
# 用法：scripts/lab.sh <子命令>
#   status           一行显示当前状态
#   idle [秒]        等到所有在展示的臂连续这么多秒（默认 8）没在跟手；最多等 120 秒，等不到返回非零
#   lend <left|right> [--force]   把一只臂借给开发程序（默认先确认空闲）。另一只臂照常展示
#   restore          恢复全量展示：让借用机械臂的开发程序退出，把臂接管回来。几秒完成
#   deploy [--force] 空闲时重启生产服务以加载主目录当前检出的代码（先跑测试）
# 环境变量 SOARM_URL 指定运行中控制台的地址，默认 http://127.0.0.1:8000
set -euo pipefail
cd "$(dirname "$0")/.."
URL="${SOARM_URL:-http://127.0.0.1:8000}"

state() { curl -sf -m 3 "$URL/api/state"; }
post() { curl -sf -m 15 -X POST "$URL$1" -H 'content-type: application/json' -d "${2:-{\}}"; }

summary() {
  state | python3 -c "$(cat <<'PY'
import json, sys
s = json.load(sys.stdin)
arms = []
for name, a in s["arms"].items():
    if a.get("lent"):
        text = "已借出"
    elif not a["connected"]:
        text = "未连接"
    else:
        text = a["status"] or "已连接"
    arms.append(name + ": " + text)
print("模式", s["mode"], " 急停", s["estop"], " 控制环", s["loop_hz"], "Hz  | ", "  ".join(arms))
PY
)"
}

wait_idle() {
  local need="${1:-8}"
  python3 - "$URL" "$need" <<'PY'
import json, sys, time, urllib.request
url, need = sys.argv[1], float(sys.argv[2])
quiet = ("等待手出现", "已借出", "已停用，保持", "已到位", "保持", "")
since, deadline = None, time.time() + 120
while time.time() < deadline:
    s = json.load(urllib.request.urlopen(url + "/api/state", timeout=3))
    busy = [n for n, a in s["arms"].items() if a["connected"] and a["status"] not in quiet]
    if busy:
        since = None
    else:
        since = since or time.time()
        if time.time() - since >= need:
            print(f"已连续 {need:.0f} 秒没有人在互动")
            sys.exit(0)
    time.sleep(0.5)
print("120 秒内一直有人在互动，先不动")
sys.exit(1)
PY
}

case "${1:-}" in
  status)
    summary
    ;;
  idle)
    wait_idle "${2:-8}"
    ;;
  lend)
    arm="${2:?用法: lab.sh lend <left|right> [--force]}"
    [ "${3:-}" = "--force" ] || wait_idle 8
    post /api/arm/lend "{\"arm\":\"$arm\"}" >/dev/null
    summary
    ;;
  restore)
    post /api/restore
    echo
    summary
    ;;
  deploy)
    [ "${2:-}" = "--force" ] || wait_idle 8
    .venv/bin/python -m pytest -q | tail -1
    scripts/production.sh stop
    scripts/production.sh start
    for _ in $(seq 1 40); do state >/dev/null 2>&1 && break; sleep 0.5; done
    sleep 8
    summary
    ;;
  *)
    sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
