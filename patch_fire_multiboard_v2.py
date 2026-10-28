#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys, re
p = "src/auto_aim_debug_mpc.cpp"
s = open(p, encoding="utf-8").read()

old = """      double w = target ? target->ekf_x()[7] : 0.0;  // EKF 旋转角速度, 无目标时 w=0
      bool fire;
      if (fabs(w) < 0.5) {
        fire = plan.fire;                       // 静止/低速靶: 直接开火
      } else {
        fire = plan.fire && plan.yaw_vel < -0.05;  // 用MPC输出速度, 不用差分噪声    // 旋转靶: 仅下降段开火
      }"""

new = """      double w = target ? target->ekf_x()[7] : 0.0;  // EKF 旋转角速度, 无目标时 w=0
      int board_count = target ? (int)target->armor_xyza_list().size() : 0;  // 同时识别到的装甲板数
      bool fire;
      if (fabs(w) < 0.5) {
        fire = plan.fire;                       // 静止/低速靶: 直接开火
      } else {
        fire = plan.fire && plan.yaw_vel < -0.05;  // 旋转靶: 仅下降段开火
      }
      if (board_count > 1) fire = false;         // 多板同时出现(误检/切换中)禁止开火"""

assert old in s, "PATTERN NOT FOUND"
s = s.replace(old, new, 1)
open(p, "w", encoding="utf-8").write(s)
print("OK")
