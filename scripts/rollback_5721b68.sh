#!/bin/bash
set -e
cd ~/Desktop/sp_vision_25_hero
echo '=== target commit ==='
git log -1 --format='%h | %ci | %s' 5721b68
echo '=== reset master to 5721b68 ==='
git reset --hard 5721b68
echo '=== verify ==='
git log --oneline -3
echo '--- status ---'
git status --short
echo '=== demo.yaml check ==='
grep -n 'max_temp_lost_count' configs/demo.yaml
echo '=== [TX->EC] frequency print present? ==='
grep -rn '实测发送频率\|send_hz\|TX->EC' src/sentry/debug_overlay.cpp | head -5