#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Fix: 回退稳定跟随门控(b95ee15)，改为 gimbal_yaw_vel < 0（云台 yaw 下降段）才允许开火
# 在 ~/srm_auto_aim 目录下执行: python3 patch_shooter_downswing.py
CPP = "tasks/auto_aim/shooter.cpp"

with open(CPP, "r", encoding="utf-8") as f:
    cpp = f.read()

OLD = """  // 稳定跟随门控：云台实际速率与目标速率匹配(±25%)才开火；
  // MPC 提前减速/回程换板时云台速率明显偏离目标速率 → 禁射
  auto target_w = targets.front().ekf_x()[7];              // EKF 目标角速度 rad/s
  auto gimbal_yaw_rate = gimbal_pos[0] - last_gimbal_yaw_; // 云台实际角速度 rad/帧
  last_gimbal_yaw_ = gimbal_pos[0];
  const double w_dead = 0.3;                               // 目标转速死区 rad/s
  const double rate_dead = 0.005;                          // 云台速率死区 rad/帧(≈0.3°/帧)
  const double rate_match_ratio = 0.25;                    // 速率匹配容差(目标速率的±25%)
  bool stable_follow =
    std::abs(target_w) > w_dead && std::abs(gimbal_yaw_rate) > rate_dead &&
    std::abs(gimbal_yaw_rate - target_w) < rate_match_ratio * std::abs(target_w);

  if (
    stable_follow &&"""

NEW = """  // 云台实际角速度（rad/帧）：gimbal_pos[0] 帧间差分
  auto gimbal_yaw_rate = gimbal_pos[0] - last_gimbal_yaw_;
  last_gimbal_yaw_ = gimbal_pos[0];
  // 只允许云台 yaw 下降段开火（gimbal_yaw_vel < 0）；
  // 上升段 = 回程换板，禁止发射；如过零抖动导致误判，可把 0.0 改为 -0.0005
  bool down_swing = gimbal_yaw_rate < 0.0;

  if (
    down_swing &&"""

if OLD not in cpp:
    print("GATE PATTERN NOT FOUND - abort, no change")
    raise SystemExit(1)
cpp = cpp.replace(OLD, NEW, 1)
with open(CPP, "w", encoding="utf-8") as f:
    f.write(cpp)
print("PATCH OK")
