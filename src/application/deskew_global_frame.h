#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "data_loader/gac_clip_root_loader.h"
#include "pose_parse/inspvax_asc_parser.h"

namespace segment_projection::application {

enum class GlobalFrame {
  kEnu,
  kUtm,
};

const char* GlobalFrameName(GlobalFrame frame);

bool ParseGlobalFrameString(const std::string& value, GlobalFrame* frame,
                            std::string* error);

bool ShouldApplyUtmConvergence(GlobalFrame frame);

std::filesystem::path SelectGnssToLidarCalibrationPath(
    const data_loader::GacClipRootLoader& clip, GlobalFrame frame);

bool RewriteSamplesForGlobalFrame(
    const pose_parse::InspvaxAscParser& parser, GlobalFrame frame,
    int64_t origin_ts_ms, std::vector<pose_parse::InspvaxSample>* samples,
    std::string* error);

}  // namespace segment_projection::application
