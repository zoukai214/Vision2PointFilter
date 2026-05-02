#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace segment_projection::projection {

enum class ImageProjectionModel {
  kUndistorted = 0,
  kRaw = 1,
};

enum class DistortionModel {
  kFisheye4 = 0,
  kDistorted8 = 1,
};

struct CameraModel {
  std::string camera_name;
  Eigen::Matrix4d T_lidar_car = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_car_lidar = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_car_cam = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_cam_car = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  std::vector<double> distortion_coeffs;
  DistortionModel raw_projection_model = DistortionModel::kFisheye4;
  int image_width = 0;
  int image_height = 0;
};

using FrontWideCameraModel = CameraModel;

struct ImageMatch {
  int64_t timestamp_ms = 0;
  int64_t delta_ms = 0;
  std::filesystem::path path;
};

}  // namespace segment_projection::projection
