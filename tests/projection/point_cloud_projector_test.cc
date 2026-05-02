#include "projection/point_cloud_projector.h"

#include <limits>
#include <iostream>

#include <opencv2/core.hpp>

namespace {

using segment_projection::data_loader::GacPcdPoint;
using segment_projection::projection::CameraModel;
using segment_projection::projection::DistortionModel;
using segment_projection::projection::ImageProjectionModel;
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
  CameraModel camera_model;
  camera_model.camera_name = "back";
  camera_model.T_car_lidar = Eigen::Matrix4d::Identity();
  camera_model.T_lidar_car = Eigen::Matrix4d::Identity();
  camera_model.T_car_cam = Eigen::Matrix4d::Identity();
  camera_model.T_cam_car = Eigen::Matrix4d::Identity();
  camera_model.K << 10.0, 0.0, 5.0, 0.0, 10.0, 5.0, 0.0, 0.0, 1.0;
  camera_model.image_width = 30;
  camera_model.image_height = 30;

  pcl::PointCloud<GacPcdPoint> cloud;
  cloud.push_back(MakePoint(0.0f, 0.0f, 2.0f, 20));
  cloud.push_back(
      MakePoint(std::numeric_limits<float>::quiet_NaN(), 0.0f, 2.0f, 25));
  cloud.push_back(
      MakePoint(0.0f, std::numeric_limits<float>::infinity(), 2.0f, 26));
  cloud.push_back(MakePoint(0.0f, 0.0f, -1.0f, 30));
  cloud.push_back(MakePoint(2.0f, 0.0f, 0.1f, 40));
  pcl::PointCloud<GacPcdPoint> raw_render_cloud;
  raw_render_cloud.push_back(MakePoint(1.0f, 0.0f, 1.0f, 10));

  ProjectionRenderConfig config;
  config.point_radius_px = 1;

  const cv::Mat input_image =
      cv::Mat::zeros(camera_model.image_height, camera_model.image_width, CV_8UC3);

  cv::Point undistorted_pixel;
  if (!segment_projection::projection::ProjectLidarPointToPixel(
          MakePoint(1.0f, 0.0f, 1.0f, 10), camera_model,
          ImageProjectionModel::kUndistorted, &undistorted_pixel) ||
      undistorted_pixel.x != 15 || undistorted_pixel.y != 5) {
    std::cerr << "expected undistorted projection to use pinhole model\n";
    return 1;
  }

  camera_model.distortion_coeffs = {0.0, 0.0, 0.0, 0.0};
  camera_model.raw_projection_model = DistortionModel::kFisheye4;
  cv::Point fisheye_pixel;
  if (!segment_projection::projection::ProjectLidarPointToPixel(
          MakePoint(1.0f, 0.0f, 1.0f, 10), camera_model,
          ImageProjectionModel::kRaw, &fisheye_pixel) ||
      fisheye_pixel.x != 12 || fisheye_pixel.y != 5) {
    std::cerr << "expected raw fisheye projection to differ from pinhole\n";
    return 1;
  }

  camera_model.distortion_coeffs = {0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  camera_model.raw_projection_model = DistortionModel::kDistorted8;
  cv::Point distorted8_pixel;
  if (!segment_projection::projection::ProjectLidarPointToPixel(
          MakePoint(1.0f, 0.0f, 1.0f, 10), camera_model,
          ImageProjectionModel::kRaw, &distorted8_pixel) ||
      distorted8_pixel.x != 20 || distorted8_pixel.y != 5) {
    std::cerr << "expected raw distorted8 projection to apply radial distortion\n";
    return 1;
  }

  cv::Mat output_image;
  int valid_count = -1;
  camera_model.distortion_coeffs = {0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  camera_model.raw_projection_model = DistortionModel::kDistorted8;
  const bool ok = segment_projection::projection::RenderProjection(
      raw_render_cloud, camera_model, ImageProjectionModel::kRaw, config,
      input_image, &output_image, &valid_count);
  if (!ok) {
    std::cerr << "RenderProjection returned false\n";
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
  if (output_image.at<cv::Vec3b>(5, 20) == cv::Vec3b(0, 0, 0)) {
    std::cerr << "expected raw render output to keep the raw pixel colored\n";
    return 1;
  }
  if (output_image.at<cv::Vec3b>(5, 15) != cv::Vec3b(0, 0, 0)) {
    std::cerr << "expected pinhole pixel to stay empty before export undistortion\n";
    return 1;
  }

  ProjectionRenderConfig invalid_config = config;
  invalid_config.intensity_color_map = "not-a-real-color-map";
  valid_count = -1;
  if (segment_projection::projection::RenderProjection(
          cloud, camera_model, ImageProjectionModel::kUndistorted,
          invalid_config, input_image, &output_image,
          &valid_count)) {
    std::cerr << "expected invalid intensity_color_map to be rejected\n";
    return 1;
  }

  return 0;
}
