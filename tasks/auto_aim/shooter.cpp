#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  if (!command.control || targets.empty() || !auto_fire_) return false;

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;
  // 方向门控：旋转靶只允许顺向跟踪开火；回程换板/过零时禁射，避免连发打空
  auto target_w = targets.front().ekf_x()[7];            // EKF 目标角速度 rad/s
  auto cmd_yaw_rate = command.yaw - last_command_.yaw;   // 指令 yaw 变化量 rad/帧
  const double w_dead = 0.3;                             // 目标转速死区 rad/s
  const double rate_dead = 0.005;                        // 指令速率死区 rad/帧(≈0.3°/帧@166fps)
  bool same_dir = std::abs(target_w) > w_dead && std::abs(cmd_yaw_rate) > rate_dead &&
                  target_w * cmd_yaw_rate > 0;

  if (
    same_dir &&
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
    aimer.debug_aim_point.valid) {
    last_command_ = command;
    return true;
  }

  last_command_ = command;
  return false;
}

}  // namespace auto_aim