#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
p = "src/auto_aim_debug_mpc.cpp"
lines = open(p, encoding="utf-8").readlines()

# 找 "bool fire;" 行, 在它前面插 board_count, 在 } 后插多板判断
fire_idx = None
for i, line in enumerate(lines):
    if "bool fire;" in line:
        fire_idx = i
        break
assert fire_idx is not None, "bool fire not found"

# 找这个 if 块的结束 } (fire_idx 后第一个以 "      }" 开头的行)
close_idx = None
for i in range(fire_idx, len(lines)):
    if lines[i].rstrip() == "      }":
        close_idx = i
        break
assert close_idx is not None, "closing brace not found"

# 在 bool fire 前插 board_count
lines.insert(fire_idx, "      int board_count = target ? (int)target->armor_xyza_list().size() : 0;  // 同时识别到的装甲板数\n")
# close_idx 因为上面 insert 偏移了 +1
close_idx += 1
# 在 } 后插多板判断
lines.insert(close_idx + 1, "      if (board_count > 1) fire = false;  // 多板同时出现(误检/切换中)禁止开火\n")

open(p, "w", encoding="utf-8").write("".join(lines))
print("OK")
