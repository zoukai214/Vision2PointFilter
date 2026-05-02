#include "projection/semantic_point_labeler.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>

#include "data_loader/gac_pcd_point.h"
#include "projection/front_wide_projection_types.h"
#include "projection/semantic_label_mapping.h"

namespace {

segment_projection::data_loader::GacPcdPoint MakePoint(float x, float y,
                                                       float z) {
  segment_projection::data_loader::GacPcdPoint point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = 1;
  point.ring = 0;
  point.point_time_offset = 0;
  return point;
}

}  // namespace

int main() {
  segment_projection::projection::FrontWideCameraModel camera_model;
  camera_model.T_car_lidar = Eigen::Matrix4d::Identity();
  camera_model.T_cam_car = Eigen::Matrix4d::Identity();
  camera_model.K << 10.0, 0.0, 5.0, 0.0, 10.0, 5.0, 0.0, 0.0, 1.0;
  camera_model.image_width = 10;
  camera_model.image_height = 10;

  cv::Mat image = cv::Mat::zeros(10, 10, CV_8UC1);
  image.at<std::uint8_t>(5, 5) = 51;
  image.at<std::uint8_t>(5, 6) = 250;

  segment_projection::projection::SemanticLabelMapping mapping;
  const auto json_path =
      std::filesystem::temp_directory_path() / "semantic_point_labeler.json";
  std::ofstream(json_path)
      << R"json({"mapping":[{"gray_value":51,"contiguous_id":13}]})json";
  if (!mapping.LoadFromJson(json_path)) {
    std::cerr << "failed to load mapping fixture\n";
    return 1;
  }

  int semantic_label = -99;
  if (!segment_projection::projection::LookupSemanticLabelForPoint(
          MakePoint(0.0f, 0.0f, 2.0f), camera_model,
          segment_projection::projection::ImageProjectionModel::kUndistorted,
          image, mapping,
          &semantic_label) ||
      semantic_label != 13) {
    std::cerr << "expected projected point to resolve to label 13\n";
    return 1;
  }

  if (!segment_projection::projection::LookupSemanticLabelForPoint(
          MakePoint(0.2f, 0.0f, 2.0f), camera_model,
          segment_projection::projection::ImageProjectionModel::kUndistorted,
          image, mapping,
          &semantic_label) ||
      semantic_label != -1) {
    std::cerr << "unknown gray value should map to -1\n";
    return 1;
  }

  if (!segment_projection::projection::LookupSemanticLabelForPoint(
          MakePoint(100.0f, 0.0f, 1.0f), camera_model,
          segment_projection::projection::ImageProjectionModel::kUndistorted,
          image, mapping,
          &semantic_label) ||
      semantic_label != -1) {
    std::cerr << "out-of-image point should map to -1\n";
    return 1;
  }

  segment_projection::projection::CameraModel fallback_camera_model =
      camera_model;
  cv::Mat first_image = cv::Mat::zeros(10, 10, CV_8UC1);
  cv::Mat second_image = cv::Mat::zeros(10, 10, CV_8UC1);
  first_image.at<std::uint8_t>(5, 5) = 250;
  second_image.at<std::uint8_t>(5, 5) = 51;
  const std::vector<segment_projection::projection::SemanticLookupContext>
      lookup_contexts = {
          {&camera_model,
           segment_projection::projection::ImageProjectionModel::kUndistorted,
           &first_image, &mapping},
          {&fallback_camera_model,
           segment_projection::projection::ImageProjectionModel::kUndistorted,
           &second_image, &mapping},
      };
  if (!segment_projection::projection::LookupSemanticLabelForPointMultiCamera(
          MakePoint(0.0f, 0.0f, 2.0f), lookup_contexts, &semantic_label) ||
      semantic_label != 13) {
    std::cerr << "multi-camera lookup should fall through to later camera\n";
    return 1;
  }

  first_image.at<std::uint8_t>(5, 5) = 51;
  second_image.at<std::uint8_t>(5, 5) = 0;
  if (!segment_projection::projection::LookupSemanticLabelForPointMultiCamera(
          MakePoint(0.0f, 0.0f, 2.0f), lookup_contexts, &semantic_label) ||
      semantic_label != 13) {
    std::cerr << "multi-camera lookup should keep first valid camera label\n";
    return 1;
  }

  return 0;
}
