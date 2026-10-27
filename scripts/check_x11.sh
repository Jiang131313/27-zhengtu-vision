#!/bin/bash
echo '== sshd X11Forwarding / DisplayOffset / X11UseLocalhost =='
grep -Ei '^\s*(X11Forwarding|X11DisplayOffset|X11UseLocalhost)' /etc/ssh/sshd_config 2>/dev/null
echo '(empty = 用默认值)'
echo
echo '== systemd sshd drop-in overrides (Ubuntu 22.04) =='
grep -rEi 'X11Forwarding' /etc/ssh/sshd_config.d/ 2>/dev/null
echo
echo '== xauth installed? =='
which xauth && xauth -V 2>&1 | head -1
echo
echo '== DISPLAY in this ssh session =='
echo "DISPLAY=${DISPLAY:-<empty>}"
echo
echo '== does NUC itself run an X server (for reference) =='
ls /tmp/.X11-unix/ 2>/dev/null
echo
echo '== graphical sessions (who) =='
who | head -5