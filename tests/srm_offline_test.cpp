// SRM 无头离线识别测试：在 demo.avi 上跑 检测->EKF跟踪->瞄准 全链路
// 不依赖 GUI（无 imshow/waitKey），输出：
//   1) 逐帧/汇总统计到 stdout（检测装甲数、跟踪状态、瞄准指令、耗时）
//   2) 标注后的视频 <input>.labeled.mp4（绿=重投影装甲板，红=瞄准点，黄=检测框）
// 用法：./srm_offline_test [config.yaml] [assets/demo/demo] [-s 0] [-e 0] [-o out.mp4]
#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | configs/demo.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                 | 视频起始帧下标 }"
  "{end-index e    | 0                 | 视频结束帧下标(0=到结尾) }"
  "{out o          |                   | 标注视频输出路径(默认<input>.labeled.mp4)}"
  "{@input-path    | assets/demo/demo  | avi和txt文件的路径(不带扩展名)}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");
  auto out_path = cli.get<std::string>("out");
  if (out_path.empty()) out_path = input_path + ".labeled.mp4";

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);
  if (!video.isOpened()) {
    fmt::print("[FATAL] cannot open video: {}\n", video_path);
    return 1;
  }
  if (!text.is_open()) {
    fmt::print("[FATAL] cannot open pose file: {}\n", text_path);
    return 1;
  }

  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  cv::Mat img;
  auto t0 = std::chrono::steady_clock::now();
  io::Command last_command;
  double last_t = -1;

  // 输出视频（先按原尺寸建，写入时缩 0.5）
  double src_fps = video.get(cv::CAP_PROP_FPS);
  if (src_fps <= 0 || src_fps > 240) src_fps = 30.0;
  int src_w = (int)video.get(cv::CAP_PROP_FRAME_WIDTH);
  int src_h = (int)video.get(cv::CAP_PROP_FRAME_HEIGHT);
  cv::Size out_size(src_w / 2, src_h / 2);
  cv::VideoWriter writer(out_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), src_fps, out_size);
  if (!writer.isOpened()) {
    // mp4v 不可用时回退 MJPG/avi
    out_path = input_path + ".labeled.avi";
    writer.open(out_path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), src_fps, out_size);
  }
  fmt::print("[INFO] video {}x{}@{:.1f}fps -> {}\n", src_w, src_h, src_fps, out_path);

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  int n_frames = 0, n_detected = 0, n_target = 0, n_shoot = 0;
  double yolo_ms_sum = 0, trk_ms_sum = 0, aim_ms_sum = 0;

  for (int frame_count = start_index;; frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;
    video.read(img);
    if (img.empty()) break;
    double t, w, x, y, z;
    if (!(text >> t >> w >> x >> y >> z)) break;
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    solver.set_R_gimbal2world({w, x, y, z});

    auto ts_yolo = std::chrono::steady_clock::now();
    auto armors = yolo.detect(img, frame_count);
    auto ts_trk = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, timestamp);
    auto ts_aim = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, timestamp, 27, false);
    auto ts_end = std::chrono::steady_clock::now();

    // 与基线一致的简化开火判据（离线无 Shooter 闭环）
    if (
      !targets.empty() && aimer.debug_aim_point.valid &&
      std::abs(command.yaw - last_command.yaw) * 57.3 < 2)
      command.shoot = true;
    if (command.control) last_command = command;

    double yolo_ms = tools::delta_time(ts_trk, ts_yolo) * 1e3;
    double trk_ms = tools::delta_time(ts_aim, ts_trk) * 1e3;
    double aim_ms = tools::delta_time(ts_end, ts_aim) * 1e3;
    yolo_ms_sum += yolo_ms;
    trk_ms_sum += trk_ms;
    aim_ms_sum += aim_ms;

    n_frames++;
    if (!armors.empty()) n_detected++;
    if (!targets.empty()) n_target++;
    if (command.shoot) n_shoot++;

    // ---- 绘制 ----
    for (const auto & armor : armors) tools::draw_points(img, armor.points, {0, 255, 255});
    tools::draw_text(
      img,
      fmt::format(
        "frame {} armors={} state={} shoot={}", frame_count, armors.size(), tracker.state(),
        command.shoot ? 1 : 0),
      {10, 30}, {0, 255, 255});
    tools::draw_text(
      img,
      fmt::format(
        "command ctrl={},yaw={:.2f},pitch={:.2f}", command.control, command.yaw * 57.3,
        command.pitch * 57.3),
      {10, 60}, {154, 50, 205});
    Eigen::Quaternion gimbal_q = {w, x, y, z};
    tools::draw_text(
      img,
      fmt::format(
        "gimbal yaw {:.2f}", (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]),
      {10, 90}, {255, 255, 255});

    if (!targets.empty()) {
      auto target = targets.front();
      if (last_t != -1) {
        for (const auto & xyza : target.armor_xyza_list()) {
          auto ip = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(img, ip, {0, 255, 0});
        }
        auto ap = aimer.debug_aim_point;
        auto ip = solver.reproject_armor(
          ap.xyza.head(3), ap.xyza[3], target.armor_type, target.name);
        if (ap.valid) tools::draw_points(img, ip, {0, 0, 255});
      }
      last_t = t;
    }

    cv::resize(img, img, {}, 0.5, 0.5);
    writer.write(img);

    if (frame_count % 25 == 0 || command.shoot)
      fmt::print(
        "[f{:>4}] armors={} targets={} state={} yaw={:.2f} pitch={:.2f} shoot={} "
        "({:.0f}/{:.0f}/{:.0f} ms)\n",
        frame_count, armors.size(), targets.size(), tracker.state(), command.yaw * 57.3,
        command.pitch * 57.3, command.shoot ? 1 : 0, yolo_ms, trk_ms, aim_ms);
  }

  writer.release();
  fmt::print("\n==== SRM offline recognition summary ====\n");
  fmt::print("frames processed      : {}\n", n_frames);
  fmt::print("frames with detection : {} ({:.1f}%)\n", n_detected, 100.0 * n_detected / std::max(1, n_frames));
  fmt::print("frames with target    : {} ({:.1f}%)\n", n_target, 100.0 * n_target / std::max(1, n_frames));
  fmt::print("frames shoot-allowed  : {}\n", n_shoot);
  if (n_frames > 0)
    fmt::print(
      "avg ms  yolo/tracker/aimer: {:.1f} / {:.1f} / {:.1f}\n", yolo_ms_sum / n_frames,
      trk_ms_sum / n_frames, aim_ms_sum / n_frames);
  fmt::print("labeled video saved   : {}\n", out_path);
  return 0;
}
