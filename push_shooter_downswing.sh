#!/bin/bash
# 在 NUC ~/srm_auto_aim 下执行: bash push_shooter_downswing.sh
cd ~/srm_auto_aim || exit 1
git add tasks/auto_aim/shooter.cpp
git commit -m "fix: Shooter 回退稳定跟随门控，改为 gimbal_yaw_vel<0 下降段才允许开火"
echo "COMMIT_EXIT=$?"
GIT_SSH_COMMAND="ssh -o BatchMode=yes -o ConnectTimeout=8" git push origin master 2>&1
echo "PUSH_EXIT=$?"
