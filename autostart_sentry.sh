#!/usr/bin/env bash
# sp_vision 开机自启守护
# 现象: 开机时 DamiaoCAN USB 适配器仍在重枚举, SN 未就绪 -> sentry 启动即崩,
#       GNOME autostart 不会重启它, 于是"自启配置了但从未成功跑起来"。
# 本脚本: 循环启动 fuck.sh, 崩溃(非零退出)自动重试, 正常退出(按 q 关窗)则退出;
#         日志超过 20MB 自动轮转。
# 回退: 删除本脚本并把 ~/.config/autostart/sp_vision.desktop 换回原样即可。

set -uo pipefail

LOG="$HOME/.cache/sp_vision_autostart.log"
MAX_LOG_BYTES=$((20 * 1024 * 1024))
MAX_FAILS=10          # 连续启动失败次数上限(每次防崩间隔 8s, 约 80s 内 USB 若仍不稳则放弃)
RETRY_SLEEP=8
PROJECT_DIR="/home/lab/Desktop/sp_vision_25_hero"
FUCK_SH="${PROJECT_DIR}/fuck.sh"

mkdir -p "$(dirname "$LOG")"
exec >>"$LOG" 2>&1

rotate_log() {
  if [[ -f "$LOG" ]]; then
    local size
    size=$(stat -c%s "$LOG" 2>/dev/null || echo 0)
    if [[ "$size" -gt "$MAX_LOG_BYTES" ]]; then
      mv -f "$LOG" "${LOG}.old" 2>/dev/null || true
      echo "===== $(date '+%F %T') [autostart] log rotated (was ${size} bytes) ====="
    fi
  fi
}

echo "===== $(date '+%F %T') [autostart] sp_vision watchdog started (pid $$) ====="

if [[ ! -x "$FUCK_SH" ]]; then
  echo "[autostart] FATAL: ${FUCK_SH} missing or not executable"
  exit 1
fi

cd "$PROJECT_DIR" || exit 1

fails=0
while true; do
  rotate_log
  echo "===== $(date '+%F %T') [autostart] attempt #$((fails + 1)): ${FUCK_SH} ====="
  "$FUCK_SH"
  rc=$?
  echo "===== $(date '+%F %T') [autostart] fuck.sh exited rc=${rc} ====="
  if [[ "$rc" -eq 0 ]]; then
    echo "[autostart] clean exit (窗口被关闭?), watchdog 结束"
    exit 0
  fi
  fails=$((fails + 1))
  if [[ "$fails" -ge "$MAX_FAILS" ]]; then
    echo "[autostart] 连续失败 ${MAX_FAILS} 次, 放弃自启 (查看上方崩溃日志)"
    exit 1
  fi
  echo "[autostart] 启动失败, ${RETRY_SLEEP}s 后重试 (fail=${fails}/${MAX_FAILS})"
  sleep "$RETRY_SLEEP"
done