#include "io/srm_link/srm_link.hpp"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                  | print this help}"
  "{@config-path   | configs/srm.yaml | yaml config path }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  tools::Exiter exiter;

  io::SrmLink srm(config_path);
  if (!srm.wait_ready(3000ms)) {
    tools::logger()->error("[cboard_test] no gimbal pose within 3s, abort");
    return 1;
  }
  tools::logger()->info("[cboard_test] SrmLink ready, reading gimbal state...");

  while (!exiter.exit()) {
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(20ms);

    Eigen::Quaterniond q = srm.q(timestamp);
    auto gs = srm.state();

    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0) * 57.3;
    tools::logger()->info(
      "z{:.2f} y{:.2f} x{:.2f} degree | bullet speed {:.2f} m/s | color(id)={} rx={} resync={}",
      eulers[0], eulers[1], eulers[2], gs.bullet_speed, gs.color, gs.rx_frame_count,
      gs.resync_count);
  }

  return 0;
}
