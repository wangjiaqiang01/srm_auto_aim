#include <fmt/core.h>

#include <chrono>
#include <csignal>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/srm_link/srm_link.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
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

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/srm.yaml | 位置参数，yaml配置文件路径 }";

using namespace std::chrono;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder(100);  //根据实际帧率调整

  io::SrmLink srm(config_path);
  if (!srm.wait_ready(std::chrono::milliseconds(3000))) {
    tools::logger()->error("[mt_auto_aim_debug] no gimbal pose within 3s, abort");
    return 1;
  }
  io::Camera camera(config_path);

  auto_aim::multithread::MultiThreadDetector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  std::mutex command_mutex;
  io::Command latest_command{false, false, 0, 0};
  auto latest_command_time = steady_clock::time_point{};
  auto last_command_log = steady_clock::time_point{};
  bool last_fire = false;
  auto_aim::multithread::CommandGener commandgener(
    shooter, aimer,
    [&](const io::Command & c) {
      srm.send(c.control, c.shoot, c.yaw, c.pitch);
      const auto now = steady_clock::now();
      {
        std::lock_guard<std::mutex> lock(command_mutex);
        latest_command = c;
        latest_command_time = now;
      }
      const bool fire = c.control && c.shoot;
      if (fire != last_fire || now - last_command_log >= 200ms) {
        tools::logger()->info(
          "[mt_auto_aim_debug] tx yaw={:.2f} deg, pitch={:.2f} deg, control={}, fire={}",
          tools::limit_rad(c.yaw) * 180.0 / CV_PI, c.pitch * 180.0 / CV_PI,
          c.control, fire);
        last_command_log = now;
      }
      last_fire = fire;
    }, plotter, true);

  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!exiter.exit()) {
      camera.read(img, t);
      detector.push(img, t);
    }
  });

  tools::logger()->info("[mt_auto_aim_debug] SrmLink ready, auto-aim loop start");

  std::string last_state;
  auto last_state_log = steady_clock::time_point{};
  while (!exiter.exit()) {
    auto t0 = std::chrono::steady_clock::now();
    // FPS 统计（1s 滑动窗口）
    static auto fps_t0 = t0;
    static int fps_cnt = 0;
    fps_cnt++;
    if (t0 - fps_t0 >= std::chrono::seconds(1)) {
      tools::logger()->info("[mt_auto_aim_debug] FPS={:.1f}",
        fps_cnt / std::chrono::duration<double>(t0 - fps_t0).count());
      fps_t0 = t0;
      fps_cnt = 0;
    }
    /// 自瞄核心逻辑
    auto [img, armors, t] = detector.debug_pop();
    Eigen::Quaterniond q = srm.q(t - 1ms);

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto targets = tracker.track(armors, t);

    commandgener.push(targets, t, srm.state().bullet_speed, ypr);  // 发送给决策线程

    const auto state = tracker.state();
    const auto now = steady_clock::now();
    if (state != last_state || now - last_state_log >= 1s) {
      tools::logger()->info(
        "[mt_auto_aim_debug] state={}, armors={}, targets={}",
        state, armors.size(), targets.size());
      last_state = state;
      last_state_log = now;
    }

    io::Command sent_command{false, false, 0, 0};
    steady_clock::time_point sent_at;
    {
      std::lock_guard<std::mutex> lock(command_mutex);
      sent_command = latest_command;
      sent_at = latest_command_time;
    }
    const bool sent_recently = sent_at != steady_clock::time_point{} &&
                               now - sent_at < 100ms;

    /// debug
    tools::draw_text(img, fmt::format("[{}]", state), {10, 30}, {255, 255, 255});
    if (sent_recently) {
      tools::draw_text(
        img, fmt::format("TX YAW {:.1f} deg", tools::limit_rad(sent_command.yaw) * 180.0 / CV_PI),
        {10, 60}, {255, 255, 255});
      if (sent_command.control && sent_command.shoot)
        tools::draw_text(img, "FIRE", {10, 95}, {0, 0, 255}, 0.9, 2);
    }

    nlohmann::json data;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      auto min_x = 1e10;
      auto & armor = armors.front();
      for (auto & a : armors) {
        if (a.center.x < min_x) {
          min_x = a.center.x;
          armor = a;
        }
      }  //always left
      solver.solve(armor);
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
    }

    if (!targets.empty()) {
      auto target = targets.front();

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255});
      else
        tools::draw_points(img, image_points, {255, 0, 0});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      static int dbg_cnt = 0;
      if (++dbg_cnt % 50 == 0)
        tools::logger()->info("EKF w={:.2f} r={:.3f} a={:.1f} last_id={}", x[7], x[8],
                              x[6] * 57.3, target.last_id);
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;

      // DS 排查第一步：原始法线角逐帧差分角速度（unwrap），用于判断前端角度是否连续
      static double prev_meas_yaw = NAN;
      static auto prev_meas_t = std::chrono::steady_clock::time_point{};
      if (!armors.empty()) {
        const double meas_yaw = target.debug_meas_yaw;
        if (!std::isnan(prev_meas_yaw)) {
          const double dt_s = std::chrono::duration<double>(t - prev_meas_t).count();
          if (dt_s > 0.001 && dt_s < 1.0) {
            double dtheta = tools::limit_rad(meas_yaw - prev_meas_yaw);
            const double raw_w = dtheta / dt_s;
            data["raw_w"] = raw_w;
            tools::logger()->info(
              "[RAW] meas={:.1f}deg dtheta={:.1f}deg dt={:.3f}s raw_w={:.2f}rad/s ekf_w={:.2f} id={} jumped={}",
              meas_yaw * 57.3, dtheta * 57.3, dt_s, raw_w, x[7], target.last_id,
              target.jumped);
          }
        }
        prev_meas_yaw = meas_yaw;
        prev_meas_t = t;
      }
      // 滤波后的当前面角（= a + last_id*90°），与 armor_yaw 同物理量可直接对比
      data["ekf_armor_yaw"] = tools::limit_rad(x[6] + target.last_id * CV_PI / 2) * 57.3;
      data["armor_yaw"] = target.debug_meas_yaw * 57.3;
      data["armor_yaw_residual"] =
        tools::limit_rad(target.debug_meas_yaw -
                         tools::limit_rad(x[6] + target.last_id * CV_PI / 2)) *
        57.3;

      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
    }

    // 云台响应情况
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = ypr[1] * 57.3;
    data["bullet_speed"] = srm.state().bullet_speed;

    plotter.plot(data);

    static auto last_ekf_log = std::chrono::steady_clock::time_point{};
    const auto ekf_now = std::chrono::steady_clock::now();
    if (ekf_now - last_ekf_log >= 200ms && !targets.empty()) {
      const auto & target = targets.front();
      const auto x = target.ekf_x();
      const auto ekf_armor_yaw =
        tools::limit_rad(x[6] + target.last_id * CV_PI / 2) * 57.3;
      const auto armor_yaw = target.debug_meas_yaw * 57.3;
      const auto residual = tools::limit_rad(
                              target.debug_meas_yaw -
                              tools::limit_rad(x[6] + target.last_id * CV_PI / 2)) *
                            57.3;
      tools::logger()->info(
        "[EKF] armor_yaw={:.2f} ekf_armor_yaw={:.2f} residual={:.2f} w={:.3f} r={:.3f} id={}",
        armor_yaw, ekf_armor_yaw, residual, x[7], x[8], target.last_id);
      last_ekf_log = ekf_now;
    }

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') {
      std::raise(SIGINT);
      break;
    }
  }

  commandgener.stop();
  detect_thread.join();

  return 0;
}
