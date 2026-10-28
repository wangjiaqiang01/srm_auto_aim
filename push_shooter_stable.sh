#!/bin/bash
# 在 NUC ~/srm_auto_aim 下执行: bash push_shooter_stable.sh
cd ~/srm_auto_aim || exit 1
git add tasks/auto_aim/shooter.cpp tasks/auto_aim/shooter.hpp
git commit -m "fix: Shooter 稳定跟随门控——云台实际速率与目标速率匹配(±25%)才开火，覆盖MPC提前减速与回程换板"
echo "COMMIT_EXIT=$?"
GIT_SSH_COMMAND="ssh -o BatchMode=yes -o ConnectTimeout=8" git push origin master 2>&1
echo "PUSH_EXIT=$?"
