#!/bin/bash
cd ~/Desktop/sp_vision_25_hero
echo '=== commits 09-21 19:40 - 20:00 (master history) ==='
git log master --pretty=format:'%h | %ci | %s' --since='2026-09-21 19:40' --until='2026-09-21 20:00'
echo
echo '=== reflog entries at 19:50-20:00 (did anything happen at 19:52?) ==='
git reflog --date=format:'%m-%d %H:%M' | grep '09-21 19:5' | head -10
echo '=== current HEAD ==='
git log --oneline -1