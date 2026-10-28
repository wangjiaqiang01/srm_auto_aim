#!/bin/bash
# 在 NUC ~/srm_auto_aim 下执行: bash push_midterm.sh
cd ~/srm_auto_aim || exit 1
git add configs/calibration.yaml configs/srm.yaml tasks/auto_aim/solver.cpp tasks/auto_aim/target.cpp
git commit -m "中期3m处打弹版"
echo "COMMIT_EXIT=$?"
GIT_SSH_COMMAND="ssh -o BatchMode=yes -o ConnectTimeout=8" git push origin master 2>&1
echo "PUSH_EXIT=$?"
