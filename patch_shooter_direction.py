#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Fix: Shooter 增加方向门控——旋转靶只允许顺向跟踪开火，回程换板/过零禁射
# 在 ~/srm_auto_aim 目录下执行: python3 patch_shooter_direction.py
PATH = "tasks/auto_aim/shooter.cpp"

with open(PATH, "r", encoding="utf-8") as f:
    src = f.read()

OLD = """  // tools::logger()->debug("d(command.yaw) is {:.4f}", std::abs(last_command_.yaw - command.yaw));
  if (
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
    aimer.debug_aim_point.valid) {"""

NEW = """  // 方向门控：旋转靶只允许顺向跟踪开火；回程换板/过零时禁射，避免连发打空
  auto target_w = targets.front().ekf_x()[7];            // EKF 目标角速度 rad/s
  auto cmd_yaw_rate = command.yaw - last_command_.yaw;   // 指令 yaw 变化量 rad/帧
  const double w_dead = 0.3;                             // 目标转速死区 rad/s
  const double rate_dead = 0.005;                        // 指令速率死区 rad/帧(≈0.3°/帧@166fps)
  bool same_dir = std::abs(target_w) > w_dead && std::abs(cmd_yaw_rate) > rate_dead &&
                  target_w * cmd_yaw_rate > 0;

  if (
    same_dir &&
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
    aimer.debug_aim_point.valid) {"""

if OLD not in src:
    print("PATTERN NOT FOUND - abort, no change")
    raise SystemExit(1)

src = src.replace(OLD, NEW, 1)
with open(PATH, "w", encoding="utf-8") as f:
    f.write(src)
print("PATCH OK")
