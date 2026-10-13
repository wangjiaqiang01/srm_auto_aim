# SRM 自瞄内外参标定流程（先拍照后标定版）

> 适用：srm_auto_aim（NUC 实机），棋盘格 12×9（内角点 11×8，方格 30mm）
> 流程特点：拍照与标定彻底分离——capture 纯拍照记录姿态，棋盘检测/解算全部由标定程序离线完成，检测失败的图自动跳过
> 最后更新：2026-10-07

## 一、总体流程

阶段A 拍照(capture) -> 阶段B 内参标定(calibrate_camera) -> 阶段C 手眼标定(calibrate_handeye) -> 阶段D 写配置(两个yaml) -> 阶段E 验证(距离读数)

- 阶段B（内参）只需 JPG，不依赖 IMU
- 阶段C（手眼）需 JPG + TXT 四元数 + 阶段B标好的内参（calibration.yaml 必须已有 camera_matrix/distort_coeffs）

## 二、前置条件

| 项 | 要求 |
|---|---|
| 硬件 | 相机 + C板(/dev/ttyACM0) + 云台上电；板子固定（手眼采集） |
| 显示 | NoMachine 图形会话（imshow 弹窗） |
| 编译 | source /opt/intel/openvino_2024.6.0/setupvars.sh && make -C build capture calibrate_camera calibrate_handeye -j$(nproc) |
| 配置 | configs/calibration.yaml: pattern_cols 11, pattern_rows 8, square_size_mm 30, com_port /dev/ttyACM0 |

曝光：标定用亮参数（8~15ms / gain 16~20，画面黑白分明）；运行 srm.yaml 保持 2ms/16（镜头几何与曝光无关）

## 三、阶段A：拍照（capture 纯拍照模式）

    cd ~/srm_auto_aim
    rm -rf assets/img_with_q
    ./build/capture configs/calibration.yaml

- 画面显示 Z/Y/X 欧拉角 + saved: N；按 s 保存 N.jpg+N.txt(四元数wxyz)，q 退出
- 拍照不做棋盘检测（流畅不卡），质量由标定阶段过滤

采集要求：
A1 内参（0.5~2m 随意移动板）：距离拉开(0.5/1/1.5/2m)、姿态覆盖四角/边缘/倾角、画面清晰不糊、15~25张
A2 手眼（板固定只转云台）：板必须固定死、距离2~3m、yaw左右30°+ pitch上下、15~20张、每张停1~2秒再按s

## 四、阶段B：内参标定

    ./build/calibrate_camera -c=configs/calibration.yaml

- 逐张弹窗按任意键翻页；failure 自动跳过
- 输出 camera_matrix / distort_coeffs + 重投影误差
- 误差：<0.6 优秀 / 0.6~1.0 可用 / >1.0 重采（曝光/模糊/姿态单一）

## 五、阶段C：手眼标定

前提：calibration.yaml 已贴内参（否则 bad conversion 崩溃）

    ./build/calibrate_handeye -c=configs/calibration.yaml

- 逐张弹窗显示 yaw/pitch/roll + 棋盘检测
- 输出 R_camera2gimbal / t_camera2gimbal + 相机偏角
- 偏角几度以内正常；90°量级 = R_gimbal2imubody 或 IMU 方向问题

## 六、阶段D：写入配置

D1 calibration.yaml：确保 camera_matrix / distort_coeffs 存在
D2 srm.yaml 第71-75行：
  71 R_gimbal2imubody: 单位阵
  72 camera_matrix: [阶段B输出]
  73 distort_coeffs: [阶段B输出]
  74 R_camera2gimbal: [阶段C输出]
  75 t_camera2gimbal: [阶段C输出]（米）

改完 grep 核对。

## 七、阶段E：验证（黄金判据：距离读数）

    ./build/mt_auto_aim_debug configs/srm.yaml

| 验证项 | 通过标准 |
|---|---|
| 距离读数 vs 卷尺 | 误差<10% |
| 检测框贴合 | 无系统性偏移 |
| 云台跟随 | 平滑、无反向 |
| 瞄准点 | 落装甲板中心 |

距离读数不对：整体偏 -> t_camera2gimbal/内参尺度；位置不同跳变 -> 畸变；方向反 -> R_camera2gimbal/IMU

## 八、常见问题

| 症状 | 原因 | 处理 |
|---|---|---|
| no gimbal pose | C板没连/没上电 | 查 /dev/ttyACM0、上电 |
| 画面暗 | 曝光太低 | 提到 8~15ms |
| 大量 failure | 模糊/暗/反光/板不全 | 删坏图重拍 |
| bad conversion | calibration.yaml 缺内参 | 先完成阶段B |
| 误差>1px | 曝光/姿态/模糊 | 加大曝光、拉开距离 |
| 偏角90°量级 | R_gimbal2imubody/IMU方向 | 检查安装与四元数 |
| 验证方向反 | R_camera2gimbal | 重跑手眼 |

## 九、当前标定值速查（2026-10-06）

内参RMS 0.8811px；fx1822.43/fy1822.76/cx743.53/cy544.63；偏角yaw-1.10/pitch2.21/roll0.38°；t_camera2gimbal [0.142,-0.004,0.041]m；运行曝光2ms/16
