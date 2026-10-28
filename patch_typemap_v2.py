#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Fix: HikRobot type_map 兼容 Mono8(0x01080001) + find 保护（缺键不崩，跳过该帧）
# 在 ~/srm_auto_aim 目录下执行: python3 patch_typemap_v2.py
PATH = "io/hikrobot/hikrobot.cpp"

with open(PATH, "r", encoding="utf-8") as f:
    src = f.read()

OLD = """      const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
      cv::cvtColor(img, dst_image, type_map.at(pixel_type));
      img = dst_image;"""

NEW = """      const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB},
        {(MvGvspPixelType)0x01080001, cv::COLOR_GRAY2RGB}};  // Mono8 灰度（相机实测输出）
      auto type_it = type_map.find(pixel_type);
      if (type_it == type_map.end()) {
        tools::logger()->warn("[HikRobot] unsupported pixel type {:#x}, frame skipped",
                              static_cast<unsigned int>(pixel_type));
        ret = MV_CC_FreeImageBuffer(handle_, &raw);
        continue;
      }
      cv::cvtColor(img, dst_image, type_it->second);
      img = dst_image;"""

if OLD not in src:
    print("PATTERN NOT FOUND - abort, no change")
    raise SystemExit(1)

src = src.replace(OLD, NEW, 1)
with open(PATH, "w", encoding="utf-8") as f:
    f.write(src)
print("PATCH OK")
