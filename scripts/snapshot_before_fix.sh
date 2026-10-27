#!/bin/bash
set -e
cd ~/Desktop/sp_vision_25_hero
mkdir -p ~/vision_backup/pre_diverged_fix
cp configs/demo.yaml src/sentry/debug_overlay.cpp src/sentry/debug_overlay.hpp src/sentry/main.cpp tasks/auto_aim/target.cpp tasks/auto_aim/target.hpp ~/vision_backup/pre_diverged_fix/
git add -A
git commit -m "snapshot: WIP before diverged-fix (rollback point)" || git -c user.name=lab -c user.email=lab@nuc commit -m "snapshot: WIP before diverged-fix (rollback point)"
git log --oneline -3
echo ---
git status --short