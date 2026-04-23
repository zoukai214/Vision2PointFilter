#include "data_loader/gac_clip_root_loader.h"

#include <fstream>

#include <glog/logging.h>
#include <jsoncpp/json/json.h>

#include "common/utils/file.h"

namespace segment_projection::data_loader {

GacClipRootLoader::GacClipRootLoader(std::filesystem::path clip_root)
    : clip_root_(std::filesystem::absolute(std::move(clip_root))) {
  SetOutputDir("output");
}

void GacClipRootLoader::SetOutputDir(std::filesystem::path output_dir) {
  if (output_dir.is_relative()) {
    output_dir_ = std::filesystem::absolute(clip_root_ / output_dir);
  } else {
    output_dir_ = std::filesystem::absolute(std::move(output_dir));
  }
}

std::filesystem::path GacClipRootLoader::CalibExtractDir() const {
  return clip_root_ / "calib_extract";
}

std::filesystem::path GacClipRootLoader::EgoRawDir() const {
  return clip_root_ / "ego_raw";
}

std::filesystem::path GacClipRootLoader::LidarRawTopDir() const {
  return clip_root_ / "lidar_raw" / "top";
}

std::filesystem::path GacClipRootLoader::GnssToLidarTopEnuPath() const {
  return CalibExtractDir() / "calib_gnss_to_lidar_top_ENU.json";
}

std::filesystem::path GacClipRootLoader::IePostTrajAscPath() const {
  return EgoRawDir() / "IE_post_traj.asc";
}

Eigen::Matrix4d GacClipRootLoader::InvertRigidTransform(
    const Eigen::Matrix4d& T) {
  Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  const Eigen::Vector3d t = T.block<3, 1>(0, 3);
  inv.block<3, 3>(0, 0) = R.transpose();
  inv.block<3, 1>(0, 3) = -R.transpose() * t;
  return inv;
}

bool GacClipRootLoader::LoadCalibMatrix(const std::filesystem::path& calib_path,
                                        const std::vector<std::string>& keys,
                                        Eigen::Matrix4d* T_out) {
  if (!T_out) {
    return false;
  }
  if (!segment_projection::common::PathExists(calib_path.string())) {
    LOG(ERROR) << "Calibration file not found: " << calib_path;
    return false;
  }

  std::ifstream ifs(calib_path);
  if (!ifs.is_open()) {
    LOG(ERROR) << "Failed to open calibration file: " << calib_path;
    return false;
  }

  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  Json::Value root;
  std::string errs;
  if (!Json::parseFromStream(builder, ifs, &root, &errs)) {
    LOG(ERROR) << "Failed to parse calibration JSON: " << errs;
    return false;
  }

  const Json::Value* node = &root;
  for (const auto& key : keys) {
    if (!node->isObject() || !node->isMember(key)) {
      LOG(ERROR) << "Missing calib key '" << key << "' in " << calib_path;
      return false;
    }
    node = &((*node)[key]);
  }

  if (!node->isArray() || node->size() != 4) {
    LOG(ERROR) << "Unexpected calib matrix format in " << calib_path;
    return false;
  }

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (Json::ArrayIndex r = 0; r < node->size(); ++r) {
    const auto& row = (*node)[r];
    if (!row.isArray() || row.size() != 4) {
      LOG(ERROR) << "Unexpected calib row size at index " << r;
      return false;
    }
    for (Json::ArrayIndex c = 0; c < row.size(); ++c) {
      T(static_cast<int>(r), static_cast<int>(c)) = row[c].asDouble();
    }
  }

  *T_out = T;
  return true;
}

bool GacClipRootLoader::LoadGnssToLidarTopEnu(
    const std::filesystem::path& calib_path, Eigen::Matrix4d* T_gnss_lidar) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  if (!LoadCalibMatrix(calib_path,
                       {"gnss-to-lidar-top", "param", "sensor_calib", "data"},
                       &T)) {
    return false;
  }
  *T_gnss_lidar = InvertRigidTransform(T);
  return true;
}

std::map<int64_t, std::filesystem::path>
GacClipRootLoader::GetPcdFilesWithTimestampsMs(
    const std::filesystem::path& lidar_dir) {
  std::map<int64_t, std::filesystem::path> pcd_files;
  if (!std::filesystem::exists(lidar_dir)) {
    LOG(ERROR) << "LiDAR directory does not exist: " << lidar_dir;
    return pcd_files;
  }

  for (const auto& entry : std::filesystem::directory_iterator(lidar_dir)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".pcd") {
      continue;
    }
    const std::string stem = entry.path().stem().string();
    const std::size_t underscore_pos = stem.find('_');
    if (underscore_pos == std::string::npos) {
      continue;
    }
    try {
      const int64_t timestamp_ns =
          static_cast<int64_t>(std::stoll(stem.substr(0, underscore_pos)));
      pcd_files[timestamp_ns / 1000000] = std::filesystem::absolute(entry.path());
    } catch (const std::exception&) {
      LOG(WARNING) << "Failed to parse timestamp from filename: " << stem;
    }
  }
  return pcd_files;
}

}  // namespace segment_projection::data_loader
