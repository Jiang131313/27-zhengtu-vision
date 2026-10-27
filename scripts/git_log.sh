#!/bin/bash
cd ~/Desktop/sp_vision_25_hero
git log --all --pretty=format:'%h | %ad | %s' --date=format:'%m-%d %H:%M' -40
echo
echo '=== branches ==='
git branch -a
echo '=== current ==='
git log --oneline -1