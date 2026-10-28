#!/bin/bash
# 在 NUC ~/srm_auto_aim 下执行: bash push_shooter.sh
cd ~/srm_auto_aim || exit 1
git add tasks/auto_aim/shooter.cpp
git commit -m "fix: Shooter 方向门控——只允许顺向跟踪开火，回程换板禁射"
echo "COMMIT_EXIT=$?"
GIT_SSH_COMMAND="ssh -o BatchMode=yes -o ConnectTimeout=8" git push origin master 2>&1
echo "PUSH_EXIT=$?"
