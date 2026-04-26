#include "projection/point_cloud_projector.h"

#include <iostream>

#include <opencv2/core.hpp>

namespace {

using segment_projection::data_loader::GacPcdPoint;
using segment_projection::projection::FrontWideCameraModel;
using segment_projection::projection::ProjectionRenderConfig;

GacPcdPoint MakePoint(float x, float y, float z, int intensity) {
  GacPcdPoint point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  point.ring = 0;
  point.point_time_offset = 0;
  return point;
}

}  // namespace

int main() {
  FrontWideCameraModel camera_model;
  camera_model.T_car_lidar = Eigen::Matrix4d::Identity();
  camera_model.T_lidar_car = Eigen::Matrix4d::Identity();
  camera_model.T_car_cam = Eigen::Matrix4d::Identity();
  camera_model.T_cam_car = Eigen::Matrix4d::Identity();
  camera_model.K << 10.0, 0.0, 5.0, 0.0, 10.0, 5.0, 0.0, 0.0, 1.0;
  camera_model.image_width = 10;
  camera_model.image_height = 10;

  pcl::PointCloud<GacPcdPoint> cloud;
  cloud.push_back(MakePoint(0.0f, 0.0f, 2.0f, 20));
  cloud.push_back(MakePoint(0.0f, 0.0f, -1.0f, 30));
  cloud.push_back(MakePoint(2.0f, 0.0f, 0.1f, 40));

  ProjectionRenderConfig config;
  config.point_radius_px = 1;

  const cv::Mat input_image =
      cv::Mat::zeros(camera_model.image_height, camera_model.image_width, CV_8UC3);
  cv::Mat output_image;
  int valid_count = -1;
  const bool ok = segment_projection::projection::RenderFrontWideProjection(
      cloud, camera_model, config, input_image, &output_image, &valid_count);
  if (!ok) {
    std::cerr << "RenderFrontWideProjection returned false\n";
    return 1;
  }
  if (valid_count != 1) {
    std::cerr << "expected exactly one valid projected point, got "
              << valid_count << "\n";
    return 1;
  }
  if (output_image.empty()) {
    std::cerr << "expected non-empty output image\n";
    return 1;
  }
  cv::Mat diff;
  cv::absdiff(output_image, input_image, diff);
  if (cv::countNonZero(diff.reshape(1)) == 0) {
    std::cerr << "expected output image to differ from input image\n";
    return 1;
  }

  return 0;
}
