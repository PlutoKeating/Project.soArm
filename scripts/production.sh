#!/usr/bin/env bash
# 以 systemd 用户服务的方式运行控制台：脱离终端常驻，异常退出自动拉起，日志进 journal。
# 用法：scripts/production.sh start|stop|restart|status|logs
#   start   启动。启动后自动进入配置里的 start_mode（生产一般设为 follow）
#   stop    停止。两只臂会先收到休息位再释放力矩
#   logs    跟随查看日志
set -euo pipefail
cd "$(dirname "$0")/.."
UNIT=soarm-console
REPO="$(pwd)"

case "${1:-}" in
  start)
    if systemctl --user is-active --quiet "$UNIT"; then
      echo "$UNIT 已在运行"; exit 0
    fi
    systemctl --user reset-failed "$UNIT" 2>/dev/null || true
    # KillSignal=SIGINT + TimeoutStopSec：给“收手并释放力矩”的退出流程留足时间
    # Restart=on-failure：只在异常退出时拉起；正常停止不会被拉起
    systemd-run --user --unit="$UNIT" --collect --quiet \
      -p Description="SO-ARM 演示控制台" \
      -p WorkingDirectory="$REPO" \
      -p Restart=on-failure -p RestartSec=3 \
      -p StartLimitIntervalSec=120 -p StartLimitBurst=5 \
      -p KillSignal=SIGINT -p TimeoutStopSec=30 \
      "$REPO/.venv/bin/soarm-console" "${@:2}"
    echo "已启动。地址见日志：scripts/production.sh logs"
    ;;
  stop)
    systemctl --user stop "$UNIT" && echo "已停止"
    ;;
  restart)
    "$0" stop || true
    "$0" start "${@:2}"
    ;;
  status)
    systemctl --user status "$UNIT" --no-pager || true
    ;;
  logs)
    journalctl --user -u "$UNIT" -f -o cat
    ;;
  *)
    echo "用法: $0 start|stop|restart|status|logs" >&2; exit 2
    ;;
esac
