"""生产与开发试验之间的交接。

展会现场随时会来人：开发程序借用一只臂做试验时，生产必须能在几秒内把它要回来。
做法是让开发程序在一个固定位置登记自己的进程号；生产要恢复全量展示时，先按登记
终止它（它收到信号后立刻松开串口、不动力矩），再重新接管机械臂。
"""

from __future__ import annotations

import os
import signal
import time
from pathlib import Path


def pidfile() -> Path:
    base = os.environ.get("XDG_RUNTIME_DIR") or "/tmp"
    return Path(base) / "soarm-lab.pid"


def claim() -> None:
    """开发程序启动时调用：登记进程号，并让 SIGTERM 像 Ctrl+C 一样触发清理。"""
    pidfile().write_text(str(os.getpid()))

    def _on_term(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _on_term)


def release() -> None:
    try:
        if pidfile().read_text().strip() == str(os.getpid()):
            pidfile().unlink()
    except OSError:
        pass


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    try:
        # 已经退出但还没被父进程回收的“僵尸”也算退出了：它不再占用串口
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0] != "Z"
    except (OSError, IndexError):
        return True


def evict(timeout_s: float = 4.0) -> str:
    """生产调用：让登记在案的开发程序退出。返回一句说明。"""
    try:
        pid = int(pidfile().read_text().strip())
    except (OSError, ValueError):
        return "没有登记在案的开发程序"
    if pid == os.getpid() or not _alive(pid):
        pidfile().unlink(missing_ok=True)
        return "没有在运行的开发程序"
    os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and _alive(pid):
        time.sleep(0.05)
    if _alive(pid):
        os.kill(pid, signal.SIGKILL)
        time.sleep(0.2)
        pidfile().unlink(missing_ok=True)
        return f"开发程序 {pid} 没有及时退出，已强制终止"
    pidfile().unlink(missing_ok=True)
    return f"开发程序 {pid} 已退出"
