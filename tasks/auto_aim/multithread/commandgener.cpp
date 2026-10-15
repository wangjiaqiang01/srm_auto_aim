#include "commandgener.hpp"

#include <algorithm>
#include <cmath>

#include <yaml-cpp/yaml.h>

#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace multithread
{

CommandGener::CommandGener(
  auto_aim::Shooter & shooter, auto_aim::Aimer & aimer,
  std::function<void(const io::Command &)> send, tools::Plotter & plotter,
  const std::string & config_path, bool debug)
: shooter_(shooter), aimer_(aimer), send_cb_(send), plotter_(plotter), stop_(false), debug_(debug)
{
  auto yaml = YAML::LoadFile(config_path);
  max_yaw_rate_ =
    (yaml["max_command_yaw_rate"] ? yaml["max_command_yaw_rate"].as<double>() : 120.0) /
    57.3;
  max_pitch_rate_ =
    (yaml["max_command_pitch_rate"] ? yaml["max_command_pitch_rate"].as<double>() : 60.0) /
    57.3;
  thread_ = std::thread(&CommandGener::generate_command, this);
}

CommandGener::~CommandGener() { stop(); }

void CommandGener::stop()
{
  if (stop_.exchange(true)) return;
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
  send_cb_({false, false, last_yaw_, last_pitch_});
}

void CommandGener::push(
  const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
  double bullet_speed, const Eigen::Vector3d & gimbal_pos)
{
  std::lock_guard<std::mutex> lock(mtx_);
  latest_ = {targets, t, bullet_speed, gimbal_pos};
  cv_.notify_one();
}

void CommandGener::generate_command()
{
  auto t0 = std::chrono::steady_clock::now();
  auto last_command_time = t0;
  bool stale_sent = false;
  while (!stop_.load()) {
    std::optional<Input> input;
    bool has_latest = false;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      has_latest = latest_.has_value();
      if (latest_ && tools::delta_time(std::chrono::steady_clock::now(), latest_->t) < 0.2) {
        input = latest_;
      } else
        input = std::nullopt;
    }
    if (input) {
      stale_sent = false;
      if (!output_initialized_) {
        last_yaw_ = input->gimbal_pos[0];
        last_pitch_ = input->gimbal_pos[1];
        output_initialized_ = true;
      }

      auto command = aimer_.aim(input->targets_, input->t, input->bullet_speed);
      bool catching_up = false;
      if (command.control && std::isfinite(command.yaw) && std::isfinite(command.pitch)) {
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::clamp(tools::delta_time(now, last_command_time), 0.0, 0.02);
        last_command_time = now;
        const double yaw_error = std::remainder(command.yaw - last_yaw_, 2.0 * CV_PI);
        const double pitch_error = command.pitch - last_pitch_;
        const double yaw_step = std::clamp(yaw_error, -max_yaw_rate_ * dt, max_yaw_rate_ * dt);
        const double pitch_step =
          std::clamp(pitch_error, -max_pitch_rate_ * dt, max_pitch_rate_ * dt);
        last_yaw_ = tools::limit_rad(last_yaw_ + yaw_step);
        last_pitch_ += pitch_step;
        command.yaw = last_yaw_;
        command.pitch = last_pitch_;
        catching_up = std::abs(yaw_error - yaw_step) > 0.5 / 57.3 ||
                      std::abs(pitch_error - pitch_step) > 0.5 / 57.3;
      } else {
        command = {false, false, last_yaw_, last_pitch_};
      }
      command.shoot = command.control && !catching_up &&
                      shooter_.shoot(command, aimer_, input->targets_, input->gimbal_pos);
      command.horizon_distance = input->targets_.empty()
                                   ? 0
                                   : std::sqrt(
                                       tools::square(input->targets_.front().ekf_x()[0]) +
                                       tools::square(input->targets_.front().ekf_x()[2]));
      send_cb_(command);
      if (debug_) {
        nlohmann::json data;
        data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
        data["cmd_yaw"] = command.yaw * 57.3;
        data["cmd_pitch"] = command.pitch * 57.3;
        data["shoot"] = command.shoot;
        data["horizon_distance"] = command.horizon_distance;
        plotter_.plot(data);
      }
    } else if (has_latest && !stale_sent) {
      send_cb_({false, false, last_yaw_, last_pitch_});
      stale_sent = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));  //approximately 500Hz
  }
}

}  // namespace multithread

}  // namespace auto_aim
