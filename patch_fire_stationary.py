#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 静止靶也能开火: yaw_vel < -0.1 改成 yaw_vel < 0.1
# 静止(|vel|~0.05)与下降段(vel<0)允许, 上升段(vel>0.1)禁止
PATH = "src/auto_aim_debug_mpc.cpp"

with open(PATH, "r", encoding="utf-8") as f:
    src = f.read()

OLD = """      // 下降段门控：仅云台 yaw 下降超过死区（gimbal_yaw_vel < -0.1）时允许开火；
      // 现场实测 yaw_vel 最大变化约 0.5，-0.1 死区滤掉过零抖动且不过度切掉顺向段；
      // 若静止靶仍轻微震动 -> 死区调大（-0.3 ~ -0.5）；若顺向段开火变稀疏 -> 调小（-0.05）
      bool fire = plan.fire && yaw_vel < -0.1;"""

NEW = """      // 开火门控: yaw_vel < 0.1 时允许开火
      //   - 静止靶: 差分噪声 |yaw_vel|~0.05 < 0.1 -> 直接开火(原 -0.1 死区把静止也禁了)
      //   - 旋转靶: 下降段 yaw_vel<0 满足; 上升段 yaw_vel>0.1 禁止(避免回程打空)
      bool fire = plan.fire && yaw_vel < 0.1;"""

if OLD not in src:
    print("PATTERN NOT FOUND - abort")
    raise SystemExit(1)
src = src.replace(OLD, NEW, 1)

with open(PATH, "w", encoding="utf-8") as f:
    f.write(src)
print("PATCH OK")
