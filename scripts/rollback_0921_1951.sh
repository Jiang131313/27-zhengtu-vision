#!/bin/bash
set -e
cd ~/Desktop/sp_vision_25_hero
echo '=== confirm target commit ==='
git log -1 --format='%h | %ci | %s' a2d5dbc
echo '=== create backup branch of current state (master=2d93845) ==='
git branch -f zt_backup_0928_fixes master
git branch --list 'zt_*'
git log --oneline -1 zt_backup_0928_fixes
echo '=== reset master to a2d5dbc ==='
git reset --hard a2d5dbc
echo '=== verify ==='
git log --oneline -3
echo '--- status ---'
git status --short
echo '--- HEAD files sanity: main.cpp / target.cpp / demo.yaml / debug_overlay.cpp exist? ---'
ls -la src/sentry/main.cpp tasks/auto_aim/target.cpp configs/demo.yaml src/sentry/debug_overlay.cpp 2>&1