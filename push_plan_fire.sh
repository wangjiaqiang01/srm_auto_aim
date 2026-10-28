#!/bin/bash
# 在 NUC ~/srm_auto_aim 下执行: bash push_plan_fire.sh
cd ~/srm_auto_aim || exit 1
git add src/auto_aim_debug_mpc.cpp
git commit -m "fix: auto_aim_debug_mpc 开火下降段门控——gimbal_yaw_vel<0 才允许下发 fire"
echo "COMMIT_EXIT=$?"
GIT_SSH_COMMAND="ssh -o BatchMode=yes -o ConnectTimeout=8" git push origin master 2>&1
echo "PUSH_EXIT=$?"
