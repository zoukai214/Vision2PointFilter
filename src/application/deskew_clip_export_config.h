#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "application/deskew_global_frame.h"

namespace segment_projection::application {

enum class ProjectionImageModel {
  kUndistorted = 0,
  kRaw = 1,
};

struct ProjectionConfig {
  bool enabled = true;
  std::string image_root_subdir = "images_seg_mask2former";
  ProjectionImageModel image_model = ProjectionImageModel::kUndistorted;
  std::vector<std::string> camera_names;
  std::string output_subdir = "projection";
  double max_time_diff_ms = 100.0;
  int point_radius_px = 2;
  std::string intensity_color_map = "turbo";
};

struct DeskewClipExportConfig {
  GlobalFrame global_frame = GlobalFrame::kEnu;
  int frame_stride = 1;
  double deskew_max_range_m = 100.0;
  double interp_max_gap_ms = 100.0;
  std::string output_subdir = "deskew_pcd";
  ProjectionConfig projection;
};

bool LoadDeskewClipExportConfig(const std::filesystem::path& config_path,
                                DeskewClipExportConfig* cfg,
                                std::string* error);

}  // namespace segment_projection::application
