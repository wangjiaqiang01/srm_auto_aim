#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Revert: 移除"识别到2块及以上装甲板禁射"（target_count），恢复 fire = plan.fire && yaw_vel < -0.1
# 只改 src/auto_aim_debug_mpc.cpp，不动 configs/srm.yaml（用户现场改动保留）
# 在 ~/srm_auto_aim 目录下执行: python3 revert_multiboard.py
PATH = "src/auto_aim_debug_mpc.cpp"

with open(PATH, "r", encoding="utf-8") as f:
    src = f.read()

# 1. 移除原子变量声明
OLD1 = """  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);
  std::atomic<int> target_count{0};  // 当前帧稳定跟踪的装甲板数量（>=2 视为换板/跨板瞬间，禁射）"""
NEW1 = """  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);"""
if OLD1 not in src:
    print("P1 NOT FOUND - abort")
    raise SystemExit(1)
src = src.replace(OLD1, NEW1, 1)

# 2. 移除检测线程写入
OLD2 = """    auto targets = tracker.track(armors, t);
    target_count.store(static_cast<int>(targets.size()));  // 供 plan_thread 多板禁射
    if (!targets.empty())"""
NEW2 = """    auto targets = tracker.track(armors, t);
    if (!targets.empty())"""
if OLD2 not in src:
    print("P2 NOT FOUND - abort")
    raise SystemExit(1)
src = src.replace(OLD2, NEW2, 1)

# 3. fire 恢复为仅 yaw_vel 门控
OLD3 = """      bool fire = plan.fire && target_count.load() < 2 && yaw_vel < -0.1;"""
NEW3 = """      bool fire = plan.fire && yaw_vel < -0.1;"""
if OLD3 not in src:
    print("P3 NOT FOUND - abort")
    raise SystemExit(1)
src = src.replace(OLD3, NEW3, 1)

with open(PATH, "w", encoding="utf-8") as f:
    f.write(src)
print("REVERT OK")
