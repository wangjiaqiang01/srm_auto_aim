#!/bin/bash
# 在 NUC ~/srm_auto_aim 下执行: bash push_fix.sh
cd ~/srm_auto_aim || exit 1
git add io/hikrobot/hikrobot.cpp
git commit -m "fix: HikRobot type_map compatible with Mono8(0x01080001) + find guard, camera frames no longer crash"
echo "COMMIT_EXIT=$?"
GIT_SSH_COMMAND="ssh -o BatchMode=yes -o ConnectTimeout=8" git push origin master 2>&1
echo "PUSH_EXIT=$?"
