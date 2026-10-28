# srm_auto_aim

上海大学 SRM 战队算法校内赛 —— 自瞄赛道参赛项目。

基于**同济大学 SuperPower 开源项目 sp-vision-25** 二次开发，目标：在 5 m 处对**绕中心旋转的 4 面装甲板（单板 140 × 125 mm）**完成"检测 → 解算 → 云台瞄准 → 开火请求"闭环。

## 来源与合规声明

- **算法基线**：同济大学 SuperPower 开源项目 [sp-vision-25](https://github.com/TongjiSuperPower)（检测 / PnP 解算 / 11 维 EKF 旋转跟踪 / Aimer 选板 / Shooter 开火判据均沿用其开源实现，按赛道要求仅做最小参数适配）。
- **通信层**：严格按照《SRM 校内赛自瞄通信协议》**从零自研**（`io/srm_link/`：2 字节小端 body_len + ID 记录、流式组帧、严格长度白名单、100 Hz 发送线程），参考
- **未使用** SRM 战队队内既有代码。
- 随仓携带：同济 demo 视频（`assets/demo/`，离线测试用）、YOLO 模型、海康 / 迈德威相机 SDK 动态库（均为开源项目随附或官方 SDK）。

## 目录结构

```
├─ assets/demo/           # demo.avi + demo.txt（同济随附，离线测试）
├─ io/srm_link/           # 【自研】SRM 官方协议通信层（FrameCodec + SrmLink）
├─ src/srm_auto_aim.cpp   # 【新增】比赛入口（单相机 + SrmLink + 检测→跟踪→瞄准→开火闭环）
├─ tests/
│   ├─ srm_link_test.cpp    # 【新增】通信层字节级单元测试（35 项，不依赖硬件）
│   └─ srm_offline_test.cpp # 【新增】无头离线全链路测试（免 GUI，输出标注视频 + 统计）
├─ configs/srm.yaml       # 【新增】比赛配置（CPU 推理、串口、颜色、标定占位）
└─ tasks/auto_aim/        # 同济算法（solver 板宽 140mm、aimer 弹速兜底 22m/s、tracker 颜色开关）
```

## 构建

环境：Ubuntu 22.04 / ROS2 Humble / OpenVINO 2024.6 / g++ 11 / cmake 3.22。

```bash
cd srm_auto_aim
cmake -B build -DCMAKE_BUILD_TYPE=Release
make -C build srm_link_test srm_offline_test srm_auto_aim -j$(nproc)
```

## 运行

### 通信层单元测试（无需硬件）

```bash
./build/srm_link_test
# 期望输出：35 项 [ OK ] + ALL TESTS PASSED
```

### 离线全链路识别（无需硬件 / 无需 GUI）

```bash
./build/srm_offline_test
# 默认读取 configs/demo.yaml + assets/demo/demo（均为程序默认值）
# 换配置/视频：./build/srm_offline_test -c=configs/demo.yaml assets/demo/demo
# 输出：控制台统计 + assets/demo/demo.labeled.mp4
# 黄框=检测、绿框=EKF 预测多板、红框=Aimer 选定瞄准点
```

### 比赛入口（需相机 + 下位机串口，上机阶段）

```bash
./build/srm_auto_aim
# 默认读取 configs/srm.yaml；自定义：-c=path/to/your.yaml
```

## 注意

- `configs/srm.yaml` 中 `camera_matrix / R_camera2gimbal / t_camera2gimbal / *_offset` 为基线占位值，**上机前必须现场标定替换**。
- 真实开火需下位机侧同时满足：右键进入视觉控制 + 左键人工发射许可 + 火控条件；上位机仅发送 `fire_flag` 开火请求。
- 离线阶段 VM 为 CPU 推理（约 7 fps），上机算力待确认，必要时以 ROI / 降分辨率 / 轻量模型提升帧率。
