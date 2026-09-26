// SrmLink 通信协议字节级单元测试（不依赖真实串口，只测 SrmFrameCodec）
// 覆盖：TX 黄金向量、RX 合法帧、半帧、粘包、非法长度重同步、非法 ID、字节序
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "io/srm_link/srm_link.hpp"

using io::SrmFrameCodec;
using io::SRM_TX_FRAME_LEN;
using io::SRM_RX_FRAME_LEN;

static int g_fail = 0;
#define CHECK(cond, msg)                                              \
  do {                                                                \
    if (!(cond)) {                                                    \
      std::printf("[FAIL] %s (line %d)\n", msg, __LINE__);            \
      ++g_fail;                                                       \
    } else {                                                          \
      std::printf("[ OK ] %s\n", msg);                                \
    }                                                                 \
  } while (0)

template <typename T>
T le_get(const std::vector<uint8_t> & f, size_t pos)
{
  T v{};
  std::memcpy(&v, f.data() + pos, sizeof(T));
  return v;
}

// 构造一帧合法下行报文（30 字节）
static std::vector<uint8_t> make_rx_frame(
  float yaw, float pitch, float roll, int32_t mode, int32_t color, float bullet_speed)
{
  std::vector<uint8_t> f;
  auto append = [&f](const void * p, size_t n) {
    auto b = static_cast<const uint8_t *>(p);
    f.insert(f.end(), b, b + n);
  };
  int16_t body_len = io::SRM_RX_BODY_LEN;
  int16_t id1 = io::SRM_GIMBAL_ID;
  int16_t id2 = io::SRM_SHOOT_ID;
  append(&body_len, 2);
  append(&id1, 2);
  io::SrmGimbalToVision g{yaw, pitch, roll, mode, color};
  append(&g, sizeof(g));
  append(&id2, 2);
  io::SrmShootToVision s{bullet_speed};
  append(&s, sizeof(s));
  return f;
}

int main()
{
  // ---------- 1. TX 帧长与零值黄金向量 ----------
  {
    auto f = SrmFrameCodec::build_tx_frame(0.0f, 0.0f, 0);
    const std::vector<uint8_t> expect = {
      16, 0,                          // body_len = 16
      1, 0,                           // ID1
      0, 0, 0, 0, 0, 0, 0, 0,         // yaw=0, pitch=0
      2, 0,                           // ID2
      0, 0, 0, 0};                    // fire_flag=0
    CHECK(f.size() == SRM_TX_FRAME_LEN, "TX frame length == 18");
    CHECK(f == expect, "TX zero frame golden bytes");
  }

  // ---------- 2. TX 数值编码（90 度 = 0x42B40000 LE: 00 00 B4 42；fire=1）----------
  {
    auto f = SrmFrameCodec::build_tx_frame(90.0f, -1.0f, 1);
    CHECK(le_get<int16_t>(f, 0) == 16, "TX body_len == 16");
    CHECK(le_get<int16_t>(f, 2) == 1, "TX first id == 1");
    CHECK(std::fabs(le_get<float>(f, 4) - 90.0f) < 1e-6, "TX yaw == 90 deg");
    CHECK(std::fabs(le_get<float>(f, 8) - (-1.0f)) < 1e-6, "TX pitch == -1 deg");
    CHECK(le_get<int16_t>(f, 12) == 2, "TX second id == 2");
    CHECK(le_get<int32_t>(f, 14) == 1, "TX fire_flag == 1");
    CHECK(f[4] == 0x00 && f[5] == 0x00 && f[6] == 0xB4 && f[7] == 0x42, "TX yaw little-endian bytes");
  }

  // ---------- 3. RX 合法帧解析 ----------
  {
    SrmFrameCodec codec;
    auto f = make_rx_frame(10.5f, -3.2f, 0.0f, 0, 101, 22.0f);
    CHECK(f.size() == SRM_RX_FRAME_LEN, "RX frame length == 30");
    auto pkts = codec.feed(f);
    CHECK(pkts.size() == 1, "RX one frame parsed");
    CHECK(pkts[0].valid, "RX packet valid");
    CHECK(std::fabs(pkts[0].gimbal.yaw - 10.5f) < 1e-5, "RX yaw == 10.5");
    CHECK(std::fabs(pkts[0].gimbal.pitch - (-3.2f)) < 1e-5, "RX pitch == -3.2");
    CHECK(pkts[0].gimbal.mode == 0, "RX mode == 0");
    CHECK(pkts[0].gimbal.color == 101, "RX color id == 101");
    CHECK(std::fabs(pkts[0].shoot.bullet_speed - 22.0f) < 1e-5, "RX bullet_speed == 22");
    CHECK(codec.buffered_bytes() == 0, "RX buffer drained");
  }

  // ---------- 4. 半帧：分两次喂入 ----------
  {
    SrmFrameCodec codec;
    auto f = make_rx_frame(1.0f, 2.0f, 3.0f, 0, 7, 18.0f);
    auto p1 = codec.feed(f.data(), 10);
    CHECK(p1.empty(), "half frame produces no packet");
    CHECK(codec.buffered_bytes() == 10, "half frame buffered 10 bytes");
    auto p2 = codec.feed(f.data() + 10, f.size() - 10);
    CHECK(p2.size() == 1 && p2[0].valid, "second half completes frame");
    CHECK(std::fabs(p2[0].gimbal.yaw - 1.0f) < 1e-5, "reassembled yaw == 1");
    CHECK(std::fabs(p2[0].shoot.bullet_speed - 18.0f) < 1e-5, "reassembled speed == 18");
  }

  // ---------- 5. 粘包：两帧连续喂入 ----------
  {
    SrmFrameCodec codec;
    auto f1 = make_rx_frame(0.0f, 0.0f, 0.0f, 0, 1, 20.0f);
    auto f2 = make_rx_frame(5.0f, 0.0f, 0.0f, 0, 1, 21.0f);
    std::vector<uint8_t> both = f1;
    both.insert(both.end(), f2.begin(), f2.end());
    auto pkts = codec.feed(both);
    CHECK(pkts.size() == 2, "two back-to-back frames parsed");
    CHECK(std::fabs(pkts[0].gimbal.yaw - 0.0f) < 1e-5, "sticky frame1 yaw");
    CHECK(std::fabs(pkts[1].gimbal.yaw - 5.0f) < 1e-5, "sticky frame2 yaw");
    CHECK(std::fabs(pkts[1].shoot.bullet_speed - 21.0f) < 1e-5, "sticky frame2 speed");
  }

  // ---------- 6. 非法 body_len 触发重同步，之后合法帧可解析 ----------
  {
    SrmFrameCodec codec;
    std::vector<uint8_t> garbage = {0x00, 0x00};  // body_len = 0，非法
    auto p0 = codec.feed(garbage);
    CHECK(p0.empty(), "invalid body_len yields no packet");
    CHECK(codec.resync_count == 1, "resync counted once");
    CHECK(codec.buffered_bytes() == 0, "buffer cleared after invalid length");
    auto f = make_rx_frame(1.0f, 1.0f, 1.0f, 0, 1, 22.0f);
    auto p1 = codec.feed(f);
    CHECK(p1.size() == 1 && p1[0].valid, "valid frame after resync parsed");
  }

  // ---------- 7. body_len 合法但 ID 非法 -> 丢弃 ----------
  {
    SrmFrameCodec codec;
    auto f = make_rx_frame(0, 0, 0, 0, 1, 22.0f);
    f[2] = 99;  // 篡改 ID1
    f[3] = 0;
    auto pkts = codec.feed(f);
    CHECK(pkts.empty(), "wrong id rejected");
    CHECK(codec.buffered_bytes() == 0, "buffer cleared after wrong id");
  }

  // ---------- 8. 1.5 帧：先到一帧+半帧，再补齐 ----------
  {
    SrmFrameCodec codec;
    auto f1 = make_rx_frame(0, 0, 0, 0, 1, 22.0f);
    auto f2 = make_rx_frame(8.0f, 0, 0, 0, 1, 22.0f);
    std::vector<uint8_t> part = f1;
    part.insert(part.end(), f2.begin(), f2.begin() + 12);  // 一帧 + 第二帧前12字节
    auto p1 = codec.feed(part);
    CHECK(p1.size() == 1, "frame plus partial -> one packet");
    CHECK(codec.buffered_bytes() == 12, "12 trailing bytes buffered");
    auto p2 = codec.feed(std::vector<uint8_t>(f2.begin() + 12, f2.end()));
    CHECK(p2.size() == 1 && std::fabs(p2[0].gimbal.yaw - 8.0f) < 1e-5, "second frame completed");
  }

  std::printf("\n==== %s ====\n", g_fail == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
  return g_fail == 0 ? 0 : 1;
}
