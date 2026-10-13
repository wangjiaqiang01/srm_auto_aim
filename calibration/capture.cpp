#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/srm_link/srm_link.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }";

void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  // 输出顺序为wxyz
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.close();
}

void capture_loop(const std::string & config_path, const std::string & output_folder)
{
  io::SrmLink srm(config_path);
  if (!srm.wait_ready(std::chrono::milliseconds(3000))) {
    tools::logger()->error("[capture] no gimbal pose within 3s, abort");
    return;
  }
  io::Camera camera(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  int count = 0;

  // ===== 纯拍照模式（先拍照、后标定）=====
  // 拍照阶段不做棋盘检测（findChessboardCorners 全图扫描几十~几百ms，会卡画面），
  // 只取流 + 记录 IMU 四元数；棋盘检测与内参/手眼解算全部由
  // calibrate_camera / calibrate_handeye 离线完成，检测失败的图会被自动跳过。
  while (true) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = srm.q(timestamp);

    // 在图像上显示欧拉角，用来判断 imuabs 系的 xyz 正方向，同时判断 imu 是否存在零漂
    auto display = img.clone();
    Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
    tools::draw_text(display, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(display, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(display, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});
    tools::draw_text(display, fmt::format("saved: {}  (photo-only, q to quit)", count), {40, 160}, {0, 255, 0});

    cv::resize(display, display, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("Press s to save, q to quit", display);
    auto key = cv::waitKey(1);
    if (key == 'q')
      break;
    else if (key != 's')
      continue;

    // 保存原始图片和对应四元数（wxyz 顺序）
    count++;
    cv::imwrite(fmt::format("{}/{}.jpg", output_folder, count), img);
    write_q(fmt::format("{}/{}.txt", output_folder, count), q);
    tools::logger()->info("[{}] Saved in {}", count, output_folder);
  }
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");

  // 新建输出文件夹
  std::filesystem::create_directory(output_folder);

  tools::logger()->info("chessboard pattern, inner-corner size from calibration.yaml");
  // 主循环，保存图片和对应四元数
  capture_loop(config_path, output_folder);

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
