#ifndef IO__SRM_LINK__SRM_LINK_HPP
#define IO__SRM_LINK__SRM_LINK_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{

// =====================================================================
// 《SRM 校内赛自瞄通信协议》报文结构体
// 全部小端编码（x86/ARM 小端平台可直接 memcpy），1 字节对齐
// =====================================================================
#pragma pack(push, 1)

// 下行（下位机->视觉）ID1：云台状态，20 字节，角度单位：度
struct SrmGimbalToVision
{
  float yaw;
  float pitch;
  float roll;
  int32_t mode;   // 协议固定为 0，视觉侧不使用
  int32_t color;  // 本方机器人 ID（字段名为历史遗留，不是颜色枚举）
};
static_assert(sizeof(SrmGimbalToVision) == 20, "SrmGimbalToVision must be 20 bytes");

// 下行 ID2：发射状态，4 字节
struct SrmShootToVision
{
  float bullet_speed;  // 裁判系统弹丸初速度 m/s
};
static_assert(sizeof(SrmShootToVision) == 4, "SrmShootToVision must be 4 bytes");

// 上行（视觉->下位机）ID1：云台目标，8 字节，角度单位：度
struct SrmVisionGimbalCommand
{
  float yaw;
  float pitch;
};
static_assert(sizeof(SrmVisionGimbalCommand) == 8, "SrmVisionGimbalCommand must be 8 bytes");

// 上行 ID2：开火请求，4 字节（非零仅表示视觉请求，真实发射由下位机安全链路决定）
struct SrmVisionShootCommand
{
  int32_t fire_flag;
};
static_assert(sizeof(SrmVisionShootCommand) == 4, "SrmVisionShootCommand must be 4 bytes");

#pragma pack(pop)

constexpr int16_t SRM_GIMBAL_ID = 1;
constexpr int16_t SRM_SHOOT_ID = 2;
constexpr int16_t SRM_TX_BODY_LEN = 16;  // 2(id)+8(gimbal)+2(id)+4(shoot)
constexpr int16_t SRM_RX_BODY_LEN = 28;  // 2(id)+20(gimbal)+2(id)+4(shoot)
constexpr size_t SRM_TX_FRAME_LEN = 2 + SRM_TX_BODY_LEN;  // 18
constexpr size_t SRM_RX_FRAME_LEN = 2 + SRM_RX_BODY_LEN;  // 30

// 弹速合法区间（协议默认 22 m/s），超出视为裁判系统未注入
constexpr double SRM_DEFAULT_BULLET_SPEED = 22.0;
constexpr double SRM_MIN_VALID_BULLET_SPEED = 10.0;
constexpr double SRM_MAX_VALID_BULLET_SPEED = 25.0;

// =====================================================================
// SrmFrameCodec：纯字节编解码，不依赖串口，供单元测试直接调用
// =====================================================================
class SrmFrameCodec
{
public:
  struct RxPacket
  {
    bool valid = false;
    SrmGimbalToVision gimbal{};
    SrmShootToVision shoot{};
  };

  // 组一帧上行报文（18 字节）。入参为角度制
  static std::vector<uint8_t> build_tx_frame(
    float yaw_deg, float pitch_deg, int32_t fire_flag);

  // 喂入从串口读到的任意长度原始字节，内部维护持久缓冲并流式组帧；
  // 每凑成一帧合法报文就返回一个 RxPacket。非法长度/ID/截断一律清空缓冲重同步
  std::vector<RxPacket> feed(const uint8_t * data, size_t len);
  std::vector<RxPacket> feed(const std::vector<uint8_t> & data)
  {
    return feed(data.data(), data.size());
  }

  void reset() { buffer_.clear(); }
  size_t buffered_bytes() const { return buffer_.size(); }
  uint64_t resync_count = 0;

private:
  std::vector<uint8_t> buffer_;
};

// 视觉侧使用的云台状态（角度已转为弧度）
struct SrmState
{
  double yaw = 0.0;          // rad
  double pitch = 0.0;        // rad
  double roll = 0.0;         // rad
  int mode = 0;              // 协议固定 0
  int color = 0;             // 本方机器人 ID 原始值
  double bullet_speed = SRM_DEFAULT_BULLET_SPEED;
  uint64_t rx_frame_count = 0;
  uint64_t resync_count = 0;
};

// =====================================================================
// SrmLink：USB CDC 虚拟串口收发线程
// - RX 线程：持久缓冲流式组帧（SrmFrameCodec），姿态入 slerp 插值队列
// - TX 线程：固定 100 Hz 发送完整 ID1+ID2 帧
// - CDC 不设置波特率；带端口枚举与断线重连
// =====================================================================
class SrmLink
{
public:
  explicit SrmLink(const std::string & config_path);
  ~SrmLink();

  // 视觉主循环调用：yaw/pitch 为弧度（与同济 Aimer 输出约定一致），内部转角度
  void send(bool control, bool fire, double yaw_rad, double pitch_rad);

  // 阻塞等待首帧云台姿态（RX 已收到至少一帧），带超时。
  // 上机启动时调用并 fail-fast：若 RX 不通，避免静默用单位姿态跑瞄准。
  bool wait_ready(std::chrono::milliseconds timeout);

  SrmState state() const;
  // 取时刻 t 的云台姿态四元数（相邻两帧 slerp 插值），接口与同济 Gimbal::q 一致
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

private:
  struct TxCommand
  {
    bool control = false;
    bool fire = false;
    double yaw_rad = 0.0;
    double pitch_rad = 0.0;
  };

  bool open_serial();
  std::vector<std::string> make_candidate_ports(const std::string & configured_port) const;
  void rx_loop();
  void tx_loop();
  void reconnect();
  void handle_packet(const SrmFrameCodec::RxPacket & pkt, std::chrono::steady_clock::time_point t);

  serial::Serial serial_;
  std::string configured_port_;
  std::vector<std::string> candidate_ports_;

  std::thread rx_thread_;
  std::thread tx_thread_;
  std::atomic<bool> quit_{false};
  bool open_failed_ = false;  // 串口全部打开失败(如C板未插)：wait_ready 立即返回 false，不再抛异常崩溃

  mutable std::mutex state_mutex_;
  SrmState state_;

  std::mutex tx_mutex_;
  TxCommand tx_command_;
  // 主循环最近一次 send() 的时间戳（微秒）。TX 线程据此做失联保护：
  // 超过 TX_HEARTBEAT_TIMEOUT_US 未刷新，自动撤销 control/fire（保留角度）。
  std::atomic<int64_t> last_send_us_{0};
  // 弹速非法告警限频（默认 1 次/秒，防止 100Hz 固件帧率刷屏）
  std::chrono::steady_clock::time_point last_speed_warn_ts_{};

  std::mutex queue_mutex_;
  std::vector<std::pair<Eigen::Quaterniond, std::chrono::steady_clock::time_point>> quat_queue_;
};

// TX 失联保护阈值。注意：VM CPU 推理实测 5~10 fps（主循环周期可达 100~270 ms），
// 因此不能按审查建议的 200ms 设——会误伤正常低频刷新；500ms 为 2 倍最坏周期，安全兜底。
constexpr int64_t TX_HEARTBEAT_TIMEOUT_US = 500000;

}  // namespace io

#endif  // IO__SRM_LINK__SRM_LINK_HPP
