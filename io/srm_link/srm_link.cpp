#include "io/srm_link/srm_link.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <stdexcept>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

// ---- 显式小端读写（不依赖平台字节序，保证协议字节确定）----
template <typename T>
void append_le(std::vector<uint8_t> & buf, const T & value)
{
  auto ptr = reinterpret_cast<const uint8_t *>(&value);
  for (size_t i = 0; i < sizeof(T); ++i) buf.push_back(ptr[i]);
}

template <typename T>
bool read_le(const std::vector<uint8_t> & buf, size_t pos, T & value)
{
  if (pos + sizeof(T) > buf.size()) return false;
  std::memcpy(&value, buf.data() + pos, sizeof(T));
  return true;
}

// 小端平台校验（x86_64 / 常见 ARM 均为小端；若移植到大端平台需改为字节拼装）
bool host_is_little_endian()
{
  const uint16_t v = 0x0001;
  return *reinterpret_cast<const uint8_t *>(&v) == 0x01;
}

std::vector<std::string> glob_ports(const std::filesystem::path & dir, const std::string & prefix)
{
  std::vector<std::string> ports;
  std::error_code ec;
  if (!std::filesystem::exists(dir, ec)) return ports;
  for (const auto & entry : std::filesystem::directory_iterator(dir, ec)) {
    if (ec) break;
    auto name = entry.path().filename().string();
    if (name.rfind(prefix, 0) == 0) ports.push_back(entry.path().string());
  }
  std::sort(ports.begin(), ports.end());
  return ports;
}
}  // namespace

// =====================================================================
// SrmFrameCodec
// =====================================================================
std::vector<uint8_t> SrmFrameCodec::build_tx_frame(
  float yaw_deg, float pitch_deg, int32_t fire_flag)
{
  std::vector<uint8_t> frame;
  frame.reserve(SRM_TX_FRAME_LEN);
  append_le<int16_t>(frame, SRM_TX_BODY_LEN);
  append_le<int16_t>(frame, SRM_GIMBAL_ID);
  append_le<float>(frame, yaw_deg);
  append_le<float>(frame, pitch_deg);
  append_le<int16_t>(frame, SRM_SHOOT_ID);
  append_le<int32_t>(frame, fire_flag);
  return frame;
}

std::vector<SrmFrameCodec::RxPacket> SrmFrameCodec::feed(const uint8_t * data, size_t len)
{
  std::vector<RxPacket> results;
  if (data == nullptr || len == 0) return results;
  buffer_.insert(buffer_.end(), data, data + len);

  while (true) {
    if (buffer_.size() < sizeof(int16_t)) break;  // 连 body_len 都不完整

    int16_t body_len = 0;
    read_le<int16_t>(buffer_, 0, body_len);

    // 严格长度白名单：下行帧 body_len 必须为 28，否则视为失同步，清空全部字节
    if (body_len != SRM_RX_BODY_LEN) {
      ++resync_count;
      buffer_.clear();
      break;
    }

    const size_t total_len = sizeof(int16_t) + SRM_RX_BODY_LEN;
    if (buffer_.size() < total_len) break;  // 帧体未到齐，等待更多字节

    std::vector<uint8_t> frame(buffer_.begin(), buffer_.begin() + total_len);
    buffer_.erase(buffer_.begin(), buffer_.begin() + total_len);

    RxPacket pkt;
    size_t pos = sizeof(int16_t);  // 跳过 body_len，指向 body
    bool ok = true;

    int16_t id1 = 0;
    ok &= read_le<int16_t>(frame, pos, id1);
    pos += sizeof(int16_t);
    if (id1 != SRM_GIMBAL_ID) ok = false;
    ok &= read_le<SrmGimbalToVision>(frame, pos, pkt.gimbal);
    pos += sizeof(SrmGimbalToVision);

    int16_t id2 = 0;
    ok &= read_le<int16_t>(frame, pos, id2);
    pos += sizeof(int16_t);
    if (id2 != SRM_SHOOT_ID) ok = false;
    ok &= read_le<SrmShootToVision>(frame, pos, pkt.shoot);
    pos += sizeof(SrmShootToVision);

    if (pos != total_len) ok = false;  // body 必须恰好消费完

    if (!ok) {
      // body_len 合法但内容非法，说明已错位，清空缓冲重同步
      ++resync_count;
      buffer_.clear();
      break;
    }

    pkt.valid = true;
    results.push_back(pkt);
  }

  return results;
}

// =====================================================================
// SrmLink
// =====================================================================
SrmLink::SrmLink(const std::string & config_path)
{
  if (!host_is_little_endian()) {
    throw std::runtime_error("[SrmLink] big-endian platform is not supported");
  }

  auto yaml = tools::load(config_path);
  configured_port_ = tools::read<std::string>(yaml, "com_port");
  candidate_ports_ = make_candidate_ports(configured_port_);

  // USB CDC 为虚拟串口，波特率无意义，不调用 setBaudrate
  serial_.setTimeout(20, 20, 0, 20, 0);

  if (!open_serial()) {
    throw std::runtime_error("[SrmLink] failed to open any serial port");
  }

  rx_thread_ = std::thread(&SrmLink::rx_loop, this);
  tx_thread_ = std::thread(&SrmLink::tx_loop, this);
  last_send_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count();
  tools::logger()->info("[SrmLink] started on {}", configured_port_);
}

SrmLink::~SrmLink()
{
  quit_ = true;
  if (rx_thread_.joinable()) rx_thread_.join();
  if (tx_thread_.joinable()) tx_thread_.join();
  try {
    if (serial_.isOpen()) serial_.close();
  } catch (...) {
  }
}

std::vector<std::string> SrmLink::make_candidate_ports(
  const std::string & configured_port) const
{
  std::vector<std::string> ports;
  auto add = [&](const std::string & p) {
    if (p.empty()) return;
    if (std::find(ports.begin(), ports.end(), p) == ports.end()) ports.push_back(p);
  };

  add(configured_port);
  add("/dev/gimbal");
  for (const auto & p : glob_ports("/dev/serial/by-id", "")) add(p);
  for (const auto & p : glob_ports("/dev", "ttyACM")) add(p);
  return ports;
}

bool SrmLink::open_serial()
{
  for (const auto & port : candidate_ports_) {
    try {
      if (serial_.isOpen()) serial_.close();
      serial_.setPort(port);
      serial_.open();
      serial_.flushInput();
      tools::logger()->info("[SrmLink] opened serial: {}", port);
      return true;
    } catch (const std::exception & e) {
      tools::logger()->warn("[SrmLink] open {} failed: {}", port, e.what());
    }
  }
  return false;
}

void SrmLink::reconnect()
{
  for (int i = 0; i < 10 && !quit_; ++i) {
    tools::logger()->warn("[SrmLink] reconnect attempt {}/10", i + 1);
    try {
      if (serial_.isOpen()) serial_.close();
    } catch (...) {
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (open_serial()) {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      quat_queue_.clear();
      tools::logger()->info("[SrmLink] reconnected");
      return;
    }
  }
}

void SrmLink::handle_packet(
  const SrmFrameCodec::RxPacket & pkt, std::chrono::steady_clock::time_point t)
{
  if (!pkt.valid) return;

  const auto & g = pkt.gimbal;
  const auto & s = pkt.shoot;

  double yaw = g.yaw * DEG_TO_RAD;
  double pitch = g.pitch * DEG_TO_RAD;
  double roll = g.roll * DEG_TO_RAD;
  Eigen::Quaterniond q =
    (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .normalized();

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    quat_queue_.emplace_back(q, t);
    // 只保留最近 50 帧姿态
    while (quat_queue_.size() > 50) quat_queue_.erase(quat_queue_.begin());
  }

  double speed = s.bullet_speed;
  bool speed_valid =
    std::isfinite(speed) && speed >= SRM_MIN_VALID_BULLET_SPEED &&
    speed <= SRM_MAX_VALID_BULLET_SPEED;
  if (!speed_valid) {
    // 限频 1 Hz 告警（下位机约 100Hz 帧率，逐帧 warn 会刷屏并拖慢 RX 线程）
    auto now = std::chrono::steady_clock::now();
    if (now - last_speed_warn_ts_ > std::chrono::seconds(1)) {
      tools::logger()->warn(
        "[SrmLink] invalid bullet_speed={:.2f}, fallback to {:.1f}", speed,
        SRM_DEFAULT_BULLET_SPEED);
      last_speed_warn_ts_ = now;
    }
    speed = SRM_DEFAULT_BULLET_SPEED;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.yaw = yaw;
  state_.pitch = pitch;
  state_.roll = roll;
  state_.mode = g.mode;
  state_.color = g.color;
  state_.bullet_speed = speed;
  ++state_.rx_frame_count;
}

void SrmLink::rx_loop()
{
  SrmFrameCodec codec;
  std::vector<uint8_t> chunk;
  int error_count = 0;

  while (!quit_) {
    if (!serial_.isOpen()) {
      reconnect();
      continue;
    }

    size_t n = 0;
    try {
      n = serial_.available();
    } catch (const std::exception &) {
      if (++error_count > 100) reconnect();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    if (n == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    chunk.resize(n);
    size_t got = 0;
    try {
      got = serial_.read(chunk.data(), n);
    } catch (const std::exception &) {
      if (++error_count > 100) reconnect();
      continue;
    }
    if (got == 0) continue;
    error_count = 0;

    auto now = std::chrono::steady_clock::now();
    auto packets = codec.feed(chunk.data(), got);
    for (const auto & pkt : packets) handle_packet(pkt, now);

    if (codec.resync_count != state_.resync_count) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_.resync_count = codec.resync_count;
      if (codec.resync_count > 0)
        tools::logger()->warn("[SrmLink] frame resync, total={}", codec.resync_count);
    }
  }
}

void SrmLink::tx_loop()
{
  while (!quit_) {
    TxCommand cmd;
    bool stale = false;
    {
      std::lock_guard<std::mutex> lock(tx_mutex_);
      cmd = tx_command_;
      auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count();
      stale = (now_us - last_send_us_.load()) > TX_HEARTBEAT_TIMEOUT_US;
    }

    // 失联保护：主循环（视觉解算）超过阈值未刷新指令——相机阻塞/线程卡死时，
    // 撤销控制与开火请求，但保留最后有效角度，避免下位机收到 0/0 甩动云台
    if (stale) {
      cmd.control = false;
      cmd.fire = false;
    }

    // 始终发送角度指令（control=false 时下位机不执行角度，仅 fire 由 control&&fire 决定）。
    // 无目标时主循环会填"上一次有效瞄准角度"，不会出现 0/0 猛甩
    float yaw_deg = static_cast<float>(tools::limit_rad(cmd.yaw_rad) * RAD_TO_DEG);
    float pitch_deg = static_cast<float>(cmd.pitch_rad * RAD_TO_DEG);
    int32_t fire_flag = (cmd.control && cmd.fire) ? 1 : 0;

    auto frame = SrmFrameCodec::build_tx_frame(yaw_deg, pitch_deg, fire_flag);
    try {
      if (serial_.isOpen()) serial_.write(frame);
    } catch (const std::exception & e) {
      tools::logger()->warn("[SrmLink] tx write failed: {}", e.what());
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100 Hz
  }
}

void SrmLink::send(bool control, bool fire, double yaw_rad, double pitch_rad)
{
  {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    tx_command_ = {control, fire, yaw_rad, pitch_rad};
  }
  last_send_us_.store(
    std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch())
      .count());
}

bool SrmLink::wait_ready(std::chrono::milliseconds timeout)
{
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!quit_) {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (!quat_queue_.empty()) return true;
    }
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

SrmState SrmLink::state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

Eigen::Quaterniond SrmLink::q(std::chrono::steady_clock::time_point t)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (quat_queue_.empty()) return Eigen::Quaterniond::Identity();
  if (quat_queue_.size() == 1) return quat_queue_.front().first;

  // t 早于最早帧 / 晚于最晚帧：直接返回边界帧
  if (t <= quat_queue_.front().second) return quat_queue_.front().first;
  if (t >= quat_queue_.back().second) return quat_queue_.back().first;

  for (size_t i = 0; i + 1 < quat_queue_.size(); ++i) {
    const auto & a = quat_queue_[i];
    const auto & b = quat_queue_[i + 1];
    if (a.second <= t && t <= b.second) {
      double tab = tools::delta_time(a.second, b.second);
      double tac = tools::delta_time(a.second, t);
      double k = tab > 1e-6 ? std::clamp(tac / tab, 0.0, 1.0) : 0.0;
      return a.first.slerp(k, b.first).normalized();
    }
  }
  return quat_queue_.back().first;
}

}  // namespace io
