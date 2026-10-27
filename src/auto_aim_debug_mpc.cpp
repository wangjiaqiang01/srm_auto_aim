#include <fmt/core.h>
#include <iostream>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/srm_link/srm_link.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/srm.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::logger()->set_level(spdlog::level::warn);  // 静音 tracker/target 内部 info，关键数据走 fmt::print

  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::SrmLink srm(config_path);
  if (!srm.wait_ready(std::chrono::milliseconds(3000))) {
    tools::logger()->error("[auto_aim_debug_mpc] no gimbal pose within 3s, abort");
    return 1;
  }
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    // SrmLink 无 yaw_vel/pitch_vel/bullet_count 反馈，gimbal_*_vel 用相邻状态差分估计
    double prev_yaw = 0.0, prev_pitch = 0.0;
    auto prev_t = t0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = srm.state();
      auto plan = planner.plan(target, gs.bullet_speed);

      auto now = std::chrono::steady_clock::now();
      double dt = tools::delta_time(now, prev_t);
      double yaw_vel = (dt > 1e-6) ? (gs.yaw - prev_yaw) / dt : 0.0;
      double pitch_vel = (dt > 1e-6) ? (gs.pitch - prev_pitch) / dt : 0.0;
      prev_yaw = gs.yaw;
      prev_pitch = gs.pitch;
      prev_t = now;

      // 下降段门控：仅云台 yaw 下降超过死区（gimbal_yaw_vel < -0.1）时允许开火；
      // 现场实测 yaw_vel 最大变化约 0.5，-0.1 死区滤掉过零抖动且不过度切掉顺向段；
      // 若静止靶仍轻微震动 → 死区调大（-0.3 ~ -0.5）；若顺向段开火变稀疏 → 调小（-0.05）
      bool fire = plan.fire && yaw_vel < 0.1;  /* 静止或下降段允许, 上升段禁止 */

      // SrmLink 协议只支持 yaw/pitch 角度目标 + fire，MPC 的 vel/acc 前馈无法下发，
      // 云台角度跟踪由下位机完成
      srm.send(plan.control, fire, plan.yaw, plan.pitch);

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = yaw_vel;      // 差分估计（SrmLink 无直接反馈）
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = pitch_vel;  // 差分估计（SrmLink 无直接反馈）

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = fire ? 1 : 0;
      data["fired"] = 0;  // SrmLink 无弹丸计数反馈，置 0

      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];   //z
        data["target_vz"] = target->ekf_x()[5];  //vz
      }

      if (target.has_value()) {
        data["w"] = target->ekf_x()[7];
      } else {
        data["w"] = 0.0;
      }

      if (target.has_value()) {
        const auto & x = target->ekf_x();
        fmt::print(
          "[MPC] r={:.3f} w={:.2f} id={} target_yaw={:.2f} plan_yaw={:.2f} gimbal_yaw={:.2f} ekf_yaw={:.2f}\n",
          x[8], x[7], target->current_id(), plan.target_yaw, plan.yaw, gs.yaw, x[6] * 57.3);
        std::cout.flush();
      }

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    auto q = srm.q(t);

    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    if (!targets.empty()) {
      auto target = targets.front();

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});
    }

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  srm.send(false, false, 0, 0);

  return 0;
}