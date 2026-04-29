#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include <Eigen/Core>

namespace segment_projection::projection {

struct CameraModel {
  std::string camera_name;
  Eigen::Matrix4d T_lidar_car = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_car_lidar = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_car_cam = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_cam_car = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
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
