#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Fix: Shooter 升级为稳定跟随门控——云台实际速率与目标速率匹配(±25%)才开火，
#     覆盖 MPC 提前减速/回程换板场景（速率明显偏离目标 → 禁射）
# 在 ~/srm_auto_aim 目录下执行: python3 patch_shooter_stable.py
import io as _io

HPP = "tasks/auto_aim/shooter.hpp"
CPP = "tasks/auto_aim/shooter.cpp"

# ---------- shooter.hpp: 增加 last_gimbal_yaw_ 成员 ----------
with open(HPP, "r", encoding="utf-8") as f:
    hpp = f.read()

HPP_OLD = """private:
  io::Command last_command_;
  double judge_distance_;
  double first_tolerance_;
  double second_tolerance_;
  bool auto_fire_;
};"""
HPP_NEW = """private:
  io::Command last_command_;
  double last_gimbal_yaw_;  // 上一帧云台 yaw（用于计算云台实际角速度，稳定跟随门控）
  double judge_distance_;
  double first_tolerance_;
  double second_tolerance_;
  bool auto_fire_;
};"""

if HPP_OLD not in hpp:
    print("HPP PATTERN NOT FOUND - abort, no change")
    raise SystemExit(1)
hpp = hpp.replace(HPP_OLD, HPP_NEW, 1)
with open(HPP, "w", encoding="utf-8") as f:
    f.write(hpp)
print("HPP PATCH OK")

# ---------- shooter.cpp: 构造初始化 + 门控替换 ----------
with open(CPP, "r", encoding="utf-8") as f:
    cpp = f.read()

CPP_OLD_INIT = "Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}"
CPP_NEW_INIT = "Shooter::Shooter(const std::string & config_path)\n: last_command_{false, false, 0, 0}, last_gimbal_yaw_{0}"
if CPP_OLD_INIT not in cpp:
    print("CPP INIT PATTERN NOT FOUND - abort, no change")
    raise SystemExit(1)
cpp = cpp.replace(CPP_OLD_INIT, CPP_NEW_INIT, 1)

CPP_OLD = """  // 方向门控：旋转靶只允许顺向跟踪开火；回程换板/过零时禁射，避免连发打空
  auto target_w = targets.front().ekf_x()[7];            // EKF 目标角速度 rad/s
  auto cmd_yaw_rate = command.yaw - last_command_.yaw;   // 指令 yaw 变化量 rad/帧
  const double w_dead = 0.3;                             // 目标转速死区 rad/s
  const double rate_dead = 0.005;                        // 指令速率死区 rad/帧(≈0.3°/帧@166fps)
  bool same_dir = std::abs(target_w) > w_dead && std::abs(cmd_yaw_rate) > rate_dead &&
                  target_w * cmd_yaw_rate > 0;

  if (
    same_dir &&"""
CPP_NEW = """  // 稳定跟随门控：云台实际速率与目标速率匹配(±25%)才开火；
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
if CPP_OLD not in cpp:
    print("CPP GATE PATTERN NOT FOUND - abort, no change")
    raise SystemExit(1)
cpp = cpp.replace(CPP_OLD, CPP_NEW, 1)
with open(CPP, "w", encoding="utf-8") as f:
    f.write(cpp)
print("CPP PATCH OK")
