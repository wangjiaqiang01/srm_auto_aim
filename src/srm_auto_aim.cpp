// SRM 校内赛自瞄赛道比赛入口
// 单相机(海康) + SrmLink(官方协议) + YOLO/传统检测 + EKF跟踪 + Aimer + Shooter
// 上行指令由 SrmLink 内部 100Hz 线程持续发送，主循环只负责视觉解算并更新最新指令
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <list>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/srm_link/srm_link.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ?       |                  | 输出命令行参数说明}"
  "{config-path c        | configs/srm.yaml | yaml 配置文件路径}"
  "{debug d              | false            | 启用调试可视化与 UDP 绘图}"
  "{record r             | false            | 录制数据}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>("config-path");
  auto debug = cli.get<bool>("debug");
  auto record = cli.get<bool>("record");

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  auto yaml = YAML::LoadFile(config_path);
  int color_id_blue_threshold =
    yaml["color_id_blue_threshold"] ? yaml["color_id_blue_threshold"].as<int>() : 100;
  auto fallback_enemy_color =
    (yaml["enemy_color"] && yaml["enemy_color"].as<std::string>() == "red")
      ? auto_aim::Color::red
      : auto_aim::Color::blue;

  // 通信（构造时打开 USB CDC，失败直接退出）
  io::SrmLink srm(config_path);
  // 启动必须先收到至少一帧下位机姿态：RX 不通则 fail-fast，
  // 避免静默用单位四元数跑瞄准（世界系错乱）
  if (!srm.wait_ready(3000ms)) {
    tools::logger()->error("[srm_auto_aim] no gimbal pose within 3s, abort");
    return 1;
  }
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_color_id = -1;

  tools::logger()->info("[srm_auto_aim] system started, config={}", config_path);

  while (!exiter.exit()) {
    camera.read(img, timestamp);

    // 下位机姿态（取帧时刻前 1ms，与同济入口一致）
    Eigen::Quaterniond q = srm.q(timestamp - 1ms);
    if (record) recorder.record(img, q, timestamp);
    solver.set_R_gimbal2world(q);
    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    // 颜色：下行 color 为本方机器人 ID；ID>=阈值视为蓝方(敌方红)，否则敌方蓝；
    // color==0（未注入）时回退配置
    auto gs = srm.state();
    auto enemy_color = fallback_enemy_color;
    if (gs.color != 0) {
      bool self_blue = gs.color >= color_id_blue_threshold;
      enemy_color = self_blue ? auto_aim::Color::red : auto_aim::Color::blue;
    }
    tracker.set_enemy_color(enemy_color);
    if (gs.color != last_color_id) {
      tools::logger()->info(
        "[srm_auto_aim] robot id={}, enemy={}", gs.color,
        enemy_color == auto_aim::Color::red ? "red" : "blue");
      last_color_id = gs.color;
    }

    // 检测 -> 跟踪 -> 瞄准
    auto armors = detector.detect(img);
    auto targets = tracker.track(armors, timestamp);

    // 无目标/失跟踪时保持"上一次有效瞄准角度"（control=false），
    // 避免向下位机发 0/0 导致视觉控制下云台猛甩 180°
    static io::Command last_valid{false, false, 0, 0};
    io::Command command{false, false, last_valid.yaw, last_valid.pitch};
    if (tracker.state() != "lost" && !targets.empty()) {
      auto aim_cmd = aimer.aim(targets, timestamp, gs.bullet_speed);
      if (aim_cmd.control) {
        last_valid = aim_cmd;
        command = aim_cmd;
      }
    }

    // 开火判据（Shooter 容差 + auto_fire 开关）；fire_flag 仅为视觉请求
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    // 更新最新指令（SrmLink TX 线程以 100Hz 持续下发）
    srm.send(command.control, command.shoot, command.yaw, command.pitch);

    if (debug) {
      auto debug_img = img.clone();
      for (const auto & armor : armors) tools::draw_points(debug_img, armor.points, {0, 255, 255});
      if (!targets.empty()) {
        auto target = targets.front();
        for (const auto & xyza : target.armor_xyza_list()) {
          auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(debug_img, image_points, {0, 255, 0});
        }
        auto aim_point = aimer.debug_aim_point;
        auto image_points = solver.reproject_armor(
          aim_point.xyza.head(3), aim_point.xyza[3], target.armor_type, target.name);
        tools::draw_points(
          debug_img, image_points, aim_point.valid ? cv::Scalar{0, 0, 255}
                                                   : cv::Scalar{255, 0, 0});
      }
      tools::draw_text(
        debug_img,
        fmt::format(
          "{} shoot={} speed={:.1f} resync={}", tracker.state(), command.shoot ? 1 : 0,
          gs.bullet_speed, gs.resync_count),
        {10, 30}, command.shoot ? cv::Scalar{0, 0, 255} : cv::Scalar{0, 255, 0});

      nlohmann::json data;
      data["gimbal_yaw"] = gimbal_pos[0] * 57.3;
      data["gimbal_pitch"] = -gimbal_pos[1] * 57.3;
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
      data["cmd_shoot"] = command.shoot ? 1 : 0;
      data["bullet_speed"] = gs.bullet_speed;
      data["robot_id"] = gs.color;
      // EKF è¯ä»·åéï¼debug onlyï¼
      // EKF debug vars, always sent, hold last value when no target
      // EKF debug vars, always sent, hold the last value when no target.
      // armor_yaw and ekf_armor_yaw refer to the same physical armor plate.
      static double last_meas_yaw = 0, last_ekf_yaw = 0, last_ekf_armor_yaw = 0;
      static double last_ekf_w = 0, last_ekf_r = 0, last_yaw_residual = 0;
      static int last_target_id = -1;
      static bool last_target_valid = false;
      last_target_valid = !targets.empty();
      if (!targets.empty()) {
        const auto & tgt = targets.front();
        const auto ekf_x = tgt.ekf_x();
        const auto ekf_armor_yaw =
          tools::limit_rad(ekf_x[6] + tgt.last_id * 2.0 * CV_PI / 4.0);
        last_meas_yaw = tgt.debug_meas_yaw * 57.3;
        last_ekf_yaw = ekf_x[6] * 57.3;
        last_ekf_armor_yaw = ekf_armor_yaw * 57.3;
        last_ekf_w = ekf_x[7];
        last_ekf_r = ekf_x[8];
        last_yaw_residual = tools::limit_rad(tgt.debug_meas_yaw - ekf_armor_yaw) * 57.3;
        last_target_id = tgt.last_id;
        last_target_valid = true;
      }
      data["target_valid"] = last_target_valid ? 1 : 0;
      data["target_id"] = last_target_id;
      data["armor_yaw"] = last_meas_yaw;
      data["ekf_armor_yaw"] = last_ekf_armor_yaw;
      data["armor_yaw_residual"] = last_yaw_residual;
      data["meas_yaw"] = last_meas_yaw;
      data["ekf_yaw"] = last_ekf_yaw;
      data["ekf_w"] = last_ekf_w;
      data["ekf_r"] = last_ekf_r;
      data["yaw_residual"] = last_yaw_residual;
      plotter.plot(data);

      static auto last_ekf_log = std::chrono::steady_clock::time_point{};
      const auto now = std::chrono::steady_clock::now();
      if (now - last_ekf_log >= 200ms) {
        tools::logger()->info(
          "[EKF] valid={} armor_yaw={:.2f} ekf_armor_yaw={:.2f} residual={:.2f} w={:.3f} "
          "r={:.3f} id={} state={}",
          last_target_valid ? 1 : 0, last_meas_yaw, last_ekf_armor_yaw, last_yaw_residual,
          last_ekf_w, last_ekf_r, last_target_id, tracker.state());
        last_ekf_log = now;
      }

      cv::resize(debug_img, debug_img, {}, 0.5, 0.5);
      cv::imshow("srm_auto_aim", debug_img);
      if (cv::waitKey(1) == 'q') break;
    }
  }

  if (debug) cv::destroyAllWindows();
  // 退出前确保停止开火请求
  srm.send(false, false, 0, 0);
  std::this_thread::sleep_for(100ms);
  return 0;
}
