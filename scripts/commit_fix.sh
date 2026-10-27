#!/bin/bash
set -e
cd ~/Desktop/sp_vision_25_hero
git add tasks/auto_aim/target.cpp configs/demo.yaml src/sentry/debug_overlay.cpp src/sentry/main.cpp
git commit -m "fix(aim): 云台识别几秒后停转 — 发散误判修复

1) Target::diverged() 按当前追踪装甲板侧取半径：
   4 装甲板且追长边(id 1/3) 用 x[8]+x[9]，其余只用 x[8]。
   旧逻辑无条件要求 x[8]+x[9]>0.05，追短边时 x[9] 自由漂移为负
   会把正常目标一帧打死（实测 r=0.444 l=-0.676 直接 diverged）。
2) demo.yaml max_temp_lost_count 15->45（约 0.75s @60fps），
   目标短暂丢失更久才判 lost。
3) debug_overlay: draw_text_panel() 之前从未被调用，Tracker state
   /Estimator 看不到；现在接入 show_debug_overlay 画在画面左下，
   丢目标时能直接看到 state 变 lost。
4) main.cpp: armors_before_tracker 之前和 after 同值（都在 track 后
   采样），改为在 tracker.track() 之前快照。" || true
git log --oneline -3
echo ---
git status --short