#!/bin/bash
export DISPLAY=192.168.1.110:0.0
setsid xclock -update 1 >/dev/null 2>&1 &
XPID=$!
sleep 8
kill $XPID 2>/dev/null
echo "xclock test window shown for 8 seconds, then closed."
exit 0