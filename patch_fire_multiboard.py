#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Fix: 识别到 2 块及以上装甲板时禁射（换板/跨板瞬间瞄点在假板上），保留 yaw_vel<-0.1 门控
# 在 ~/srm_auto_aim 目录下执行: python3 patch_fire_multiboard.py
PATH = "src/auto_aim_debug_mpc.cpp"

with open(PATH, "r", encoding="utf-8") as f:
    src = f.read()

# ---------- 1. 声明原子变量 ----------
OLD1 = """  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);"""
NEW1 = """  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);
  std::atomic<int> target_count{0};  // 当前帧稳定跟踪的装甲板数量（>=2 视为换板/跨板瞬间，禁射）"""
if OLD1 not in src:
    print("P1 NOT FOUND - abort")
    raise SystemExit(1)
src = src.replace(OLD1, NEW1, 1)

# ---------- 2. 检测线程写入板数量 ----------
OLD2 = """    auto targets = tracker.track(armors, t);
    if (!targets.empty())
      target_queue.push(targets.front());"""
NEW2 = """    auto targets = tracker.track(armors, t);
    target_count.store(static_cast<int>(targets.size()));  // 供 plan_thread 多板禁射
    if (!targets.empty())
      target_queue.push(targets.front());"""
if OLD2 not in src:
    print("P2 NOT FOUND - abort")
    raise SystemExit(1)
src = src.replace(OLD2, NEW2, 1)

# ---------- 3. fire 门控增加多板禁射 ----------
OLD3 = """      bool fire = plan.fire && yaw_vel < -0.1;"""
NEW3 = """      bool fire = plan.fire && target_count.load() < 2 && yaw_vel < -0.1;"""
if OLD3 not in src:
    print("P3 NOT FOUND - abort")
    raise SystemExit(1)
src = src.replace(OLD3, NEW3, 1)

with open(PATH, "w", encoding="utf-8") as f:
    f.write(src)
print("PATCH OK")
