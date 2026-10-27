#!/bin/bash
cd ~/Desktop/sp_vision_25_hero
echo '=== tags ==='
git tag -l
echo '=== stash ==='
git stash list
echo '=== reflog (last 40) ==='
git reflog --date=format:'%m-%d %H:%M' -40
echo '=== all dangling / unreachable commits? (fsck) ==='
git fsck --no-reflogs --unreachable 2>/dev/null | head -20