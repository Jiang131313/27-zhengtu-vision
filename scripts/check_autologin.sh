#!/bin/bash
echo '== 默认启动目标 =='
systemctl get-default 2>/dev/null
echo
echo '== 图形会话进程 =='
ps -eo pid,user,cmd | grep -E 'Xorg|gdm3|gdm |lightdm' | grep -v grep | head -8
echo
echo '== lab 的图形登录会话(DISPLAY/Type) =='
loginctl list-sessions 2>/dev/null
echo
echo '== Xauthority 位置 =='
ls -la /home/lab/.Xauthority 2>/dev/null
ls -la /run/user/1000/gdm/Xauthority 2>/dev/null
echo "XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-<empty>}"
echo
echo '== tty2 上有登录进程吗 =='
ps -eo pid,user,tty,cmd | awk '$3=="tty2"' | head -5
echo
echo '== lab 用户名下已开的显示进程(同会话) =='
find /proc -maxdepth 2 -name environ 2>/dev/null | while read f; do
  pid=$(echo "$f" | cut -d/ -f3)
  grep -qz 'DISPLAY=:0' "$f" 2>/dev/null && echo "$pid $(cat /proc/$pid/comm 2>/dev/null) [DISPLAY=:0]"
done | head -8