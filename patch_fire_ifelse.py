#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
p = "src/auto_aim_debug_mpc.cpp"
s = open(p, encoding="utf-8").read()

old = """      double w = target ? target->ekf_x()[7] : 0.0;  // EKF 旋转角速度, 无目标时 w=0
      bool fire = plan.fire && (fabs(w) < 0.1 || yaw_vel < -0.05);"""

# 兼容多种可能的旧写法
import re
pattern = re.compile(
    r"      double w = target \? target->ekf_x\(\)\[7\] : 0\.0;.*\n"
    r".*fire.*\n"
)
new = """      double w = target ? target->ekf_x()[7] : 0.0;  // EKF 旋转角速度, 无目标时 w=0
      bool fire;
      if (fabs(w) < 0.5) {
        fire = plan.fire;                       // 静止/低速靶: 直接开火
      } else {
        fire = plan.fire && yaw_vel < -0.05;    // 旋转靶: 仅下降段开火
      }
"""

m = pattern.search(s)
if not m:
    # 打印实际内容以便排查
    for i, line in enumerate(s.splitlines()):
        if "double w" in line or "bool fire" in line:
            print(f"FOUND {i+1}: {line!r}")
    print("PATTERN NOT MATCHED")
    sys.exit(1)

s = s[:m.start()] + new + s[m.end():]
open(p, "w", encoding="utf-8").write(s)
print("PATCH OK")
