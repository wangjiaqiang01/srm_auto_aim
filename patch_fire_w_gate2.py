#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 按行号替换 fire 门控 (第81行, 0-indexed=80)
PATH = "src/auto_aim_debug_mpc.cpp"

with open(PATH, "r", encoding="utf-8") as f:
    lines = f.readlines()

# 找到 bool fire 那一行
fire_idx = None
for i, line in enumerate(lines):
    if "bool fire = plan.fire" in line:
        fire_idx = i
        break

if fire_idx is None:
    print("fire line not found")
    raise SystemExit(1)

# 在 fire 行前插入 double w = ..., 并替换 fire 行
new_lines = lines[:fire_idx]
new_lines.append("      double w = target->ekf_x()[7];  // EKF 旋转角速度(rad/s)\n")
new_lines.append("      // |w|<0.5 静止/低速 -> 直接 fire; 旋转 -> 仅下降段 yaw_vel<-0.05 fire\n")
new_lines.append("      bool fire = plan.fire && (fabs(w) < 0.5 || yaw_vel < -0.05);\n")
new_lines.extend(lines[fire_idx+1:])

with open(PATH, "w", encoding="utf-8") as f:
    f.writelines(new_lines)
print(f"PATCH OK, replaced line {fire_idx+1}")
