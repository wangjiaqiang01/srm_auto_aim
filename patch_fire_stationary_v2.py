#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# fire 门控修正: 静止特判加连续帧计数, 避免旋转过零瞬间误触发
PATH = "src/auto_aim_debug_mpc.cpp"

with open(PATH, "r", encoding="utf-8") as f:
    src = f.read()

OLD = """      // 下降段门控：仅云台 yaw 下降超过死区（gimbal_yaw_vel < -0.1）时允许开火；
      // 现场实测 yaw_vel 最大变化约 0.5，-0.1 死区滤掉过零抖动且不过度切掉顺向段；
      // 若静止靶仍轻微震动 -> 死区调大（-0.3 ~ -0.5）；若顺向段开火变稀疏 -> 调小（-0.05）
      bool fire = plan.fire && (yaw_vel < -0.05 || fabs(yaw_vel) < 0.05);  /* 下降段或静止(|vel|<0.05)允许, 上升段禁止 */  /* 下降段<0.05允许, 上升段禁止 */"""

NEW = """      // 开火门控:
      //   - 旋转下降段: yaw_vel < -0.05 -> 允许
      //   - 静止靶: |yaw_vel| < 0.05 且连续 >= 10 帧(约0.2s) -> 允许(用持续帧区分旋转过零的短暂穿越)
      //   - 上升段/换向瞬间: 禁止
      static int stationary_cnt = 0;
      if (fabs(yaw_vel) < 0.05) stationary_cnt++;
      else                       stationary_cnt = 0;
      bool fire = plan.fire && (yaw_vel < -0.05 || stationary_cnt >= 10);"""

if OLD not in src:
    print("PATTERN NOT FOUND")
    raise SystemExit(1)
src = src.replace(OLD, NEW, 1)

with open(PATH, "w", encoding="utf-8") as f:
    f.write(src)
print("PATCH OK")
