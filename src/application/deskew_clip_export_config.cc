#include "application/deskew_clip_export_config.h"

#include <set>
#include <string>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace segment_projection::application {
namespace {

bool ParseProjectionConfig(const YAML::Node& node, ProjectionConfig* projection,
                           std::string* error) {
  if (!projection) {
    if (error) {
      *error = "projection is null";
    }
    return false;
  }

  if (node["enabled"]) {
    projection->enabled = node["enabled"].as<bool>();
  }
  if (node["image_root_subdir"]) {
    projection->image_root_subdir = node["image_root_subdir"].as<std::string>();
  }
  if (node["camera_names"]) {
    const YAML::Node camera_names = node["camera_names"];
    if (!camera_names.IsSequence()) {
      if (error) {
        *error = "projection.camera_names must be a sequence";
      }
      return false;
    }
    projection->camera_names.clear();
    for (const YAML::Node& item : camera_names) {
      if (!item.IsScalar()) {
        if (error) {
          *error = "projection.camera_names must contain strings";
        }
        return false;
      }
      projection->camera_names.push_back(item.as<std::string>());
    }
  }
  if (node["output_subdir"]) {
    projection->output_subdir = node["output_subdir"].as<std::string>();
  }
  if (node["max_time_diff_ms"]) {
    projection->max_time_diff_ms = node["max_time_diff_ms"].as<double>();
  }
  if (node["point_radius_px"]) {
    projection->point_radius_px = node["point_radius_px"].as<int>();
  }
  if (node["intensity_color_map"]) {
    projection->intensity_color_map =
        node["intensity_color_map"].as<std::string>();
  }

  if (!projection->enabled) {
    return true;
  }
  if (projection->image_root_subdir.empty()) {
    if (error) {
      *error = "projection.image_root_subdir must not be empty when enabled";
    }
    return false;
  }
  if (!node["camera_names"]) {
    if (error) {
      *error = "projection.camera_names must be set when enabled";
    }
    return false;
  }
  if (projection->camera_names.empty()) {
    if (error) {
      *error = "projection.camera_names must contain at least one camera";
    }
    return false;
  }

  std::set<std::string> seen_names;
  for (const std::string& camera_name : projection->camera_names) {
    if (!seen_names.insert(camera_name).second) {
      if (error) {
        *error = "projection.camera_names contains duplicate entry: " +
                 camera_name;
      }
      return false;
    }
  }
  if (projection->output_subdir.empty()) {
    if (error) {
      *error = "projection.output_subdir must not be empty when enabled";
    }
    return false;
  }
  if (projection->max_time_diff_ms < 0.0) {
    if (error) {
      *error = "projection.max_time_diff_ms must be >= 0";
    }
    return false;
  }
  if (projection->point_radius_px <= 0) {
    if (error) {
      *error = "projection.point_radius_px must be > 0";
    }
    return false;
  }
  if (projection->intensity_color_map.empty()) {
    if (error) {
      *error = "projection.intensity_color_map must not be empty";
    }
    return false;
  }
  return true;
}

}  // namespace

bool LoadDeskewClipExportConfig(const std::filesystem::path& config_path,
                                DeskewClipExportConfig* cfg,
                                std::string* error) {
  if (!cfg) {
    if (error) {
      *error = "cfg is null";
    }
    return false;
  }

  *cfg = DeskewClipExportConfig{};
  try {
    const YAML::Node root = YAML::LoadFile(config_path.string());
    const YAML::Node node = root["deskew_clip_export"];
    if (!node) {
      if (error) {
        *error = "Missing 'deskew_clip_export' root in " +
                 config_path.string();
      }
      return false;
    }

    if (node["frame_stride"]) {
      cfg->frame_stride = node["frame_stride"].as<int>();
    }
    if (const YAML::Node pose = node["pose"]) {
      if (pose["coord_frame"]) {
        std::string frame_error;
        if (!ParseGlobalFrameString(pose["coord_frame"].as<std::string>(),
                                    &cfg->global_frame, &frame_error)) {
          if (error) {
            *error = frame_error;
          }
          return false;
        }
      }
    }
    if (const YAML::Node deskew = node["deskew"]) {
      if (deskew["max_range_m"]) {
        cfg->deskew_max_range_m = deskew["max_range_m"].as<double>();
      }
      if (deskew["interp_max_gap_ms"]) {
        cfg->interp_max_gap_ms = deskew["interp_max_gap_ms"].as<double>();
      }
    }
    if (const YAML::Node output = node["output"]) {
      if (output["subdir"]) {
        cfg->output_subdir = output["subdir"].as<std::string>();
      }
    }
    if (const YAML::Node projection = node["projection"]) {
      if (!ParseProjectionConfig(projection, &cfg->projection, error)) {
        return false;
      }
    }

    if (cfg->frame_stride <= 0) {
      if (error) {
        *error = "frame_stride must be > 0";
      }
      return false;
    }
    if (!(cfg->interp_max_gap_ms > 0.0)) {
      if (error) {
        *error = "deskew.interp_max_gap_ms must be > 0";
      }
      return false;
    }
  } catch (const YAML::Exception& e) {
    if (error) {
      *error = e.what();
    }
    return false;
  } catch (const std::exception& e) {
    if (error) {
      *error = e.what();
    }
    return false;
  }

  return true;
}

}  // namespace segment_projection::application
