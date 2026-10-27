#!/bin/bash
echo '=== 自启日志 ==='
ls -la ~/.cache/sp_vision_autostart.log 2>/dev/null
if [ -f ~/.cache/sp_vision_autostart.log ]; then
  tail -40 ~/.cache/sp_vision_autostart.log
else
  echo '无自启日志'
fi
echo
echo '=== autostart desktop 文件时间 ==='
stat -c '%y %n' ~/.config/autostart/sp_vision.desktop 2>/dev/null
echo
echo '=== 是否有别的 sentry 相关自启残留 ==='
grep -rl 'sentry' ~/.config/autostart/ 2>/dev/null
echo
echo '=== 当前 sentry 进程 ==='
pgrep -af 'build/sentry' | grep -v pgrep || echo '未运行'
echo
echo '=== 上次开机时间 ==='
who -b 2>/dev/null
echo
echo '=== 日志目录权限 ==='
ls -la ~/.cache/ | grep -i autostart || echo 'no autostart log yet'