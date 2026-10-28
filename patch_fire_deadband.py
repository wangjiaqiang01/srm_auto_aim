#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Fix: fire 门控死区 -0.5 → -0.1（用户现场观察 yaw_vel 最大变化约 0.5）
# 在 ~/srm_auto_aim 目录下执行: python3 patch_fire_deadband.py
PATH = "src/auto_aim_debug_mpc.cpp"

with open(PATH, "r", encoding="utf-8") as f:
    src = f.read()

OLD = """      // 下降段门控：仅云台 yaw 明显下降（gimbal_yaw_vel < -0.5）时允许开火；
      // 负死区把静止靶差分噪声区间（|yaw_vel| < 0.5）整体禁射，避免 fire 位高频翻转导致云台震动；
      // 旋转靶下降段 yaw_vel≈-3 正常开火；回程上升段 >0 禁射
      // 若静止靶仍轻微震动 → 死区调大（-0.8 ~ -1.0）；若顺向段开火变稀疏 → 调小（-0.2）
      bool fire = plan.fire && yaw_vel < -0.5;"""

NEW = """      // 下降段门控：仅云台 yaw 下降超过死区（gimbal_yaw_vel < -0.1）时允许开火；
      // 现场实测 yaw_vel 最大变化约 0.5，-0.1 死区滤掉过零抖动且不过度切掉顺向段；
      // 若静止靶仍轻微震动 → 死区调大（-0.3 ~ -0.5）；若顺向段开火变稀疏 → 调小（-0.05）
      bool fire = plan.fire && yaw_vel < -0.1;"""

if OLD not in src:
    print("PATTERN NOT FOUND - abort, no change")
    raise SystemExit(1)
src = src.replace(OLD, NEW, 1)
with open(PATH, "w", encoding="utf-8") as f:
    f.write(src)
print("PATCH OK")
