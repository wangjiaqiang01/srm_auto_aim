#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Fix: auto_aim_debug_mpc 开火门控——仅云台 yaw 下降段（gimbal_yaw_vel < 0）允许开火
# 在 ~/srm_auto_aim 目录下执行: python3 patch_plan_fire_gate.py
PATH = "src/auto_aim_debug_mpc.cpp"

with open(PATH, "r", encoding="utf-8") as f:
    src = f.read()

OLD = """      auto target = target_queue.front();
      auto gs = srm.state();
      auto plan = planner.plan(target, gs.bullet_speed);

      // SrmLink 协议只支持 yaw/pitch 角度目标 + fire，MPC 的 vel/acc 前馈无法下发，
      // 云台角度跟踪由下位机完成
      srm.send(plan.control, plan.fire, plan.yaw, plan.pitch);

      auto now = std::chrono::steady_clock::now();
      double dt = tools::delta_time(now, prev_t);
      double yaw_vel = (dt > 1e-6) ? (gs.yaw - prev_yaw) / dt : 0.0;
      double pitch_vel = (dt > 1e-6) ? (gs.pitch - prev_pitch) / dt : 0.0;
      prev_yaw = gs.yaw;
      prev_pitch = gs.pitch;
      prev_t = now;"""

NEW = """      auto target = target_queue.front();
      auto gs = srm.state();
      auto plan = planner.plan(target, gs.bullet_speed);

      auto now = std::chrono::steady_clock::now();
      double dt = tools::delta_time(now, prev_t);
      double yaw_vel = (dt > 1e-6) ? (gs.yaw - prev_yaw) / dt : 0.0;
      double pitch_vel = (dt > 1e-6) ? (gs.pitch - prev_pitch) / dt : 0.0;
      prev_yaw = gs.yaw;
      prev_pitch = gs.pitch;
      prev_t = now;

      // 下降段门控：仅云台 yaw 下降（gimbal_yaw_vel < 0）时允许开火，上升段/回程换板禁射
      // 若过零抖动导致误判，可把 0.0 改为 -0.001（更严格）或 +0.001（更宽松）
      bool fire = plan.fire && yaw_vel < 0.0;

      // SrmLink 协议只支持 yaw/pitch 角度目标 + fire，MPC 的 vel/acc 前馈无法下发，
      // 云台角度跟踪由下位机完成
      srm.send(plan.control, fire, plan.yaw, plan.pitch);"""

if OLD not in src:
    print("MAIN PATTERN NOT FOUND - abort, no change")
    raise SystemExit(1)
src = src.replace(OLD, NEW, 1)

OLD2 = """      data["fire"] = plan.fire ? 1 : 0;"""
NEW2 = """      data["fire"] = fire ? 1 : 0;"""
if OLD2 not in src:
    print("PLOT PATTERN NOT FOUND - abort, no change")
    raise SystemExit(1)
src = src.replace(OLD2, NEW2, 1)

with open(PATH, "w", encoding="utf-8") as f:
    f.write(src)
print("PATCH OK")
