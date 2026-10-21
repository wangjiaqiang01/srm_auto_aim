#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path)
: last_command_{false, false, 0, 0}, last_gimbal_yaw_{0}
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
  // 稳定跟随门控：云台实际速率与目标速率匹配(±25%)才开火；
  // MPC 提前减速/回程换板时云台速率明显偏离目标速率 → 禁射
  auto target_w = targets.front().ekf_x()[7];              // EKF 目标角速度 rad/s
  auto gimbal_yaw_rate = gimbal_pos[0] - last_gimbal_yaw_; // 云台实际角速度 rad/帧
  last_gimbal_yaw_ = gimbal_pos[0];
  const double w_dead = 0.3;                               // 目标转速死区 rad/s
  const double rate_dead = 0.005;                          // 云台速率死区 rad/帧(≈0.3°/帧)
  const double rate_match_ratio = 0.25;                    // 速率匹配容差(目标速率的±25%)
  bool stable_follow =
    std::abs(target_w) > w_dead && std::abs(gimbal_yaw_rate) > rate_dead &&
    std::abs(gimbal_yaw_rate - target_w) < rate_match_ratio * std::abs(target_w);

  if (
    stable_follow &&
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