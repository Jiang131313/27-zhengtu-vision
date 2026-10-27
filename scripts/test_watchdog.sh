#!/bin/bash
# 受控实测: 模拟 GNOME 自启环境启动守护脚本, 观察能否拉起 sentry
export DISPLAY=:0
export XDG_RUNTIME_DIR=/run/user/1000
LOG=/home/lab/.cache/sp_vision_autostart.log
rm -f "$LOG"
setsid bash /home/lab/Desktop/sp_vision_25_hero/autostart_sentry.sh >/dev/null 2>&1 &
WATCHDOG_PID=$!
echo "watchdog pid=$WATCHDOG_PID"
echo "waiting 25s for build + start..."
sleep 25
echo "=== watchdog 进程 ==="
pgrep -af 'autostart_sentry' | grep -v pgrep || echo 'watchdog 已退出(可能成功退出或失败)'
echo "=== sentry 进程 ==="
pgrep -af 'build/sentry' | grep -v pgrep | head -3 || echo 'sentry 未在运行'
echo "=== 日志尾部 ==="
tail -25 "$LOG" 2>/dev/null
echo "=== 清理测试进程 ==="
pkill -f 'autostart_sentry.sh' 2>/dev/null
pkill -f 'build/sentry' 2>/dev/null
sleep 1
pgrep -af 'build/sentry' | grep -v pgrep || echo 'sentry 已清理'