#!/bin/bash
cd ~/Desktop/sp_vision_25_hero
echo '=== demo.yaml max_temp_lost_count (expect 15 for rolled-back version) ==='
grep -n 'max_temp_lost_count' configs/demo.yaml
echo '=== target.cpp diverged() diff vs HEAD ==='
git diff HEAD --stat
echo '=== confirm working tree clean ==='
git status --short | head