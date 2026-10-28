#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# fire 门控: 用 EKF 的 w(旋转角速度) 区分静止/旋转
#   |w| < 0.5 rad/s -> 静止靶 -> 直接 fire
#   |w| >= 0.5      -> 旋转靶 -> 仅下降段 yaw_vel < -0.05 fire
PATH = "src/auto_aim_debug_mpc.cpp"

with open(PATH, "r", encoding="utf-8") as f:
    src = f.read()

OLD = """      // 下降段门控：仅云台 yaw 下降超过死区（gimbal_yaw_vel < -0.1）时允许开火；
      // 现场实测 yaw_vel 最大变化约 0.5，-0.1 死区滤掉过零抖动且不过度切掉顺向段；
      // 若静止靶仍轻微震动 -> 死区调大（-0.3 ~ -0.5）；若顺向段开火变稀疏 -> 调小（-0.05）
      bool fire = plan.fire && (yaw_vel < -0.05 || fabs(yaw_vel) < 0.05);  /* 下降段或静止(|vel|<0.05)允许, 上升段禁止 */  /* 下降段<0.05允许, 上升段禁止 */"""

NEW = """      // 开火门控: 用 EKF 的 w(旋转角速度) 区分静止/旋转
      //   |w| < 0.5 rad/s -> 静止/低速靶 -> 直接 fire(不被 yaw_vel 门控拦)
      //   |w| >= 0.5      -> 旋转靶     -> 仅下降段 yaw_vel < -0.05 允许
      double w = target->ekf_x()[7];
      bool fire = plan.fire && (fabs(w) < 0.5 || yaw_vel < -0.05);"""

if OLD not in src:
    print("PATTERN NOT FOUND")
    raise SystemExit(1)
src = src.replace(OLD, NEW, 1)

with open(PATH, "w", encoding="utf-8") as f:
    f.write(src)
print("PATCH OK")
