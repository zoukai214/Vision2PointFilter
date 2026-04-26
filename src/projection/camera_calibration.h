#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <jsoncpp/json/json.h>

#include "data_loader/gac_clip_root_loader.h"
#include "projection/front_wide_projection_types.h"

namespace segment_projection::projection {
namespace detail {

inline bool LoadJson(const std::filesystem::path& path, Json::Value* root) {
  if (!root) {
    return false;
  }
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    LOG(ERROR) << "Failed to open JSON file: " << path;
    return false;
  }

  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  std::string errs;
  if (!Json::parseFromStream(builder, ifs, root, &errs)) {
    LOG(ERROR) << "Failed to parse JSON file " << path << ": " << errs;
    return false;
  }
  return true;
}

inline const Json::Value* FindNode(const Json::Value& root,
                                   const std::vector<std::string>& keys) {
  const Json::Value* node = &root;
  for (const auto& key : keys) {
    if (!node->isObject() || !node->isMember(key)) {
      return nullptr;
    }
    node = &((*node)[key]);
  }
  return node;
}

inline bool LoadMatrix3d(const Json::Value& root,
                         const std::vector<std::string>& keys,
                         Eigen::Matrix3d* matrix) {
  if (!matrix) {
    return false;
  }
  const Json::Value* node = FindNode(root, keys);
  if (!node || !node->isArray() || node->size() != 3) {
    LOG(ERROR) << "Expected 3x3 matrix at requested JSON path";
    return false;
  }

  Eigen::Matrix3d value = Eigen::Matrix3d::Identity();
  for (Json::ArrayIndex r = 0; r < node->size(); ++r) {
    const auto& row = (*node)[r];
    if (!row.isArray() || row.size() != 3) {
      LOG(ERROR) << "Expected 3 entries in row " << r;
      return false;
    }
    for (Json::ArrayIndex c = 0; c < row.size(); ++c) {
      value(static_cast<int>(r), static_cast<int>(c)) = row[c].asDouble();
    }
  }

  *matrix = value;
  return true;
}

inline bool LoadInt(const Json::Value& root, const std::vector<std::string>& keys,
                    int* value) {
  if (!value) {
    return false;
  }
  const Json::Value* node = FindNode(root, keys);
  if (!node || !node->isInt()) {
    LOG(ERROR) << "Expected integer at requested JSON path";
    return false;
  }
  *value = node->asInt();
  return true;
}

}  // namespace detail

inline bool LoadFrontWideCameraModel(
    const std::filesystem::path& lidar_top_to_car_path,
    const std::filesystem::path& camera_front_wide_to_car_path,
    FrontWideCameraModel* model) {
  if (!model) {
    return false;
  }

  FrontWideCameraModel loaded_model;
  if (!segment_projection::data_loader::GacClipRootLoader::LoadCalibMatrix(
          lidar_top_to_car_path,
          {"lidar-top-to-car", "param", "sensor_calib", "data"},
          &loaded_model.T_car_lidar)) {
    return false;
  }
  loaded_model.T_lidar_car =
      segment_projection::data_loader::GacClipRootLoader::InvertRigidTransform(
          loaded_model.T_car_lidar);

  if (!segment_projection::data_loader::GacClipRootLoader::LoadCalibMatrix(
          camera_front_wide_to_car_path,
          {"camera-front-wide-to-car-undistort", "param", "sensor_calib",
           "data"},
          &loaded_model.T_car_cam)) {
    return false;
  }
  loaded_model.T_cam_car =
      segment_projection::data_loader::GacClipRootLoader::InvertRigidTransform(
          loaded_model.T_car_cam);

  Json::Value root;
  if (!detail::LoadJson(camera_front_wide_to_car_path, &root)) {
    return false;
  }
  if (!detail::LoadMatrix3d(root,
                            {"camera-front-wide-undistort", "param",
                             "cam_matrix", "data"},
                            &loaded_model.K)) {
    return false;
  }
  if (!detail::LoadInt(root,
                       {"camera-front-wide-undistort", "param", "width"},
                       &loaded_model.image_width)) {
    return false;
  }
  if (!detail::LoadInt(root,
                       {"camera-front-wide-undistort", "param", "height"},
                       &loaded_model.image_height)) {
    return false;
  }
  if (loaded_model.image_width <= 0 || loaded_model.image_height <= 0) {
    LOG(ERROR) << "Invalid front-wide image size: "
               << loaded_model.image_width << "x"
               << loaded_model.image_height;
    return false;
  }

  *model = loaded_model;
  return true;
}

}  // namespace segment_projection::projection
