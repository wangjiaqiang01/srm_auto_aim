#!/bin/bash
# 在 NUC ~/srm_auto_aim 下执行: bash push_fire_deadband.sh
cd ~/srm_auto_aim || exit 1
git add src/auto_aim_debug_mpc.cpp
git commit -m "fix: fire 门控死区 0 改为 -0.5——静止靶差分噪声区间整体禁射，消除 fire 位高频翻转致云台震动"
echo "COMMIT_EXIT=$?"
GIT_SSH_COMMAND="ssh -o BatchMode=yes -o ConnectTimeout=8" git push origin master 2>&1
echo "PUSH_EXIT=$?"
