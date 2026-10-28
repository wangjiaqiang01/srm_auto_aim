#!/bin/bash
# 在 NUC ~/srm_auto_aim 下执行: bash push_multiboard.sh
cd ~/srm_auto_aim || exit 1
git add src/auto_aim_debug_mpc.cpp
git commit -m "feat: 识别到2块及以上装甲板时禁射（换板/跨板瞬间瞄点在假板上），叠加 yaw_vel<-0.1 门控"
echo "COMMIT_EXIT=$?"
GIT_SSH_COMMAND="ssh -o BatchMode=yes -o ConnectTimeout=8" git push origin master 2>&1
echo "PUSH_EXIT=$?"
