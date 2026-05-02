#include "projection/point_cloud_projector.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <opencv2/imgproc.hpp>

namespace segment_projection::projection {
namespace {

struct ProjectedPoint {
  cv::Point pixel;
  int intensity = 0;
};

bool IsFinitePoint(const Eigen::Vector3d& point) {
  return std::isfinite(point.x()) && std::isfinite(point.y()) &&
         std::isfinite(point.z());
}

bool IsSupportedColorMap(const std::string& intensity_color_map) {
  return intensity_color_map == "turbo";
}

cv::Mat PrepareOutputImage(const cv::Mat& input_image) {
  if (input_image.empty()) {
    return cv::Mat();
  }
  if (input_image.type() == CV_8UC3) {
    return input_image.clone();
  }

  cv::Mat output_image;
  if (input_image.type() == CV_8UC1) {
    cv::cvtColor(input_image, output_image, cv::COLOR_GRAY2BGR);
    return output_image;
  }
  if (input_image.type() == CV_8UC4) {
    cv::cvtColor(input_image, output_image, cv::COLOR_BGRA2BGR);
    return output_image;
  }
  return cv::Mat();
}

cv::Scalar PseudoColor(double normalized_intensity) {
  const double t = std::clamp(normalized_intensity, 0.0, 1.0);

  if (t < 0.33) {
    const double local = t / 0.33;
    return cv::Scalar(255.0 * (1.0 - local), 255.0 * local, 0.0);
  }
  if (t < 0.66) {
    const double local = (t - 0.33) / 0.33;
    return cv::Scalar(0.0, 255.0, 255.0 * local);
  }

  const double local = (t - 0.66) / 0.34;
  return cv::Scalar(0.0, 255.0 * (1.0 - local), 255.0);
}

bool ProjectPoint(const Eigen::Vector3d& point_cam,
                  const CameraModel& camera_model,
                  ImageProjectionModel image_model,
                  cv::Point* pixel) {
  if (!pixel || !IsFinitePoint(point_cam) || point_cam.z() <= 0.0) {
    return false;
  }

  const double fx = camera_model.K(0, 0);
  const double fy = camera_model.K(1, 1);
  const double cx = camera_model.K(0, 2);
  const double cy = camera_model.K(1, 2);
  const double x = point_cam.x() / point_cam.z();
  const double y = point_cam.y() / point_cam.z();

  double x_distorted = x;
  double y_distorted = y;
  if (image_model == ImageProjectionModel::kRaw) {
    constexpr double kEpsilon = 1e-12;
    if (camera_model.distortion_coeffs.empty()) {
      return false;
    }
    if (camera_model.raw_projection_model == DistortionModel::kFisheye4) {
      const double r = std::sqrt(x * x + y * y);
      if (r > kEpsilon) {
        const double theta = std::atan(r);
        const double theta2 = theta * theta;
        const double k1 = camera_model.distortion_coeffs.size() > 0
                              ? camera_model.distortion_coeffs[0]
                              : 0.0;
        const double k2 = camera_model.distortion_coeffs.size() > 1
                              ? camera_model.distortion_coeffs[1]
                              : 0.0;
        const double k3 = camera_model.distortion_coeffs.size() > 2
                              ? camera_model.distortion_coeffs[2]
                              : 0.0;
        const double k4 = camera_model.distortion_coeffs.size() > 3
                              ? camera_model.distortion_coeffs[3]
                              : 0.0;
        const double theta4 = theta2 * theta2;
        const double theta6 = theta4 * theta2;
        const double theta8 = theta4 * theta4;
        const double theta_distorted =
            theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 +
                     k4 * theta8);
        const double scale = theta_distorted / r;
        x_distorted = x * scale;
        y_distorted = y * scale;
      }
    } else {
      const double r2 = x * x + y * y;
      const double r4 = r2 * r2;
      const double r6 = r4 * r2;
      const double k1 = camera_model.distortion_coeffs.size() > 0
                            ? camera_model.distortion_coeffs[0]
                            : 0.0;
      const double k2 = camera_model.distortion_coeffs.size() > 1
                            ? camera_model.distortion_coeffs[1]
                            : 0.0;
      const double p1 = camera_model.distortion_coeffs.size() > 2
                            ? camera_model.distortion_coeffs[2]
                            : 0.0;
      const double p2 = camera_model.distortion_coeffs.size() > 3
                            ? camera_model.distortion_coeffs[3]
                            : 0.0;
      const double k3 = camera_model.distortion_coeffs.size() > 4
                            ? camera_model.distortion_coeffs[4]
                            : 0.0;
      const double k4 = camera_model.distortion_coeffs.size() > 5
                            ? camera_model.distortion_coeffs[5]
                            : 0.0;
      const double k5 = camera_model.distortion_coeffs.size() > 6
                            ? camera_model.distortion_coeffs[6]
                            : 0.0;
      const double k6 = camera_model.distortion_coeffs.size() > 7
                            ? camera_model.distortion_coeffs[7]
                            : 0.0;
      const double radial_numerator = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
      const double radial_denominator = 1.0 + k4 * r2 + k5 * r4 + k6 * r6;
      if (std::abs(radial_denominator) <= kEpsilon) {
        return false;
      }
      const double radial = radial_numerator / radial_denominator;
      const double xy = x * y;
      x_distorted = x * radial + 2.0 * p1 * xy + p2 * (r2 + 2.0 * x * x);
      y_distorted = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * xy;
    }
  }

  const double u = fx * x_distorted + cx;
  const double v = fy * y_distorted + cy;
  if (!std::isfinite(u) || !std::isfinite(v)) {
    return false;
  }
  if (u < 0.0 || u >= static_cast<double>(camera_model.image_width) ||
      v < 0.0 || v >= static_cast<double>(camera_model.image_height)) {
    return false;
  }

  pixel->x = static_cast<int>(u);
  pixel->y = static_cast<int>(v);
  return true;
}

}  // namespace

bool ProjectLidarPointToPixel(
    const segment_projection::data_loader::GacPcdPoint& point,
    const CameraModel& camera_model, ImageProjectionModel image_model,
    cv::Point* pixel) {
  if (!pixel || !std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z)) {
    return false;
  }

  const Eigen::Vector4d point_lidar(point.x, point.y, point.z, 1.0);
  const Eigen::Vector4d point_car = camera_model.T_car_lidar * point_lidar;
  const Eigen::Vector4d point_cam = camera_model.T_cam_car * point_car;
  return ProjectPoint(point_cam.head<3>(), camera_model, image_model, pixel);
}

bool RenderProjection(
    const pcl::PointCloud<segment_projection::data_loader::GacPcdPoint>& cloud,
    const CameraModel& camera_model, ImageProjectionModel image_model,
    const ProjectionRenderConfig& config, const cv::Mat& input_image,
    cv::Mat* output_image, int* valid_projected_count) {
  if (!output_image || !valid_projected_count || config.point_radius_px <= 0 ||
      !IsSupportedColorMap(config.intensity_color_map) ||
      camera_model.image_width <= 0 || camera_model.image_height <= 0 ||
      input_image.cols != camera_model.image_width ||
      input_image.rows != camera_model.image_height) {
    return false;
  }
  if (image_model == ImageProjectionModel::kRaw &&
      camera_model.distortion_coeffs.empty()) {
    return false;
  }

  cv::Mat rendered_image = PrepareOutputImage(input_image);
  if (rendered_image.empty()) {
    return false;
  }

  std::vector<ProjectedPoint> projected_points;
  projected_points.reserve(cloud.size());

  int min_intensity = std::numeric_limits<int>::max();
  int max_intensity = std::numeric_limits<int>::lowest();

  for (const auto& point : cloud) {
    cv::Point pixel;
    if (!ProjectLidarPointToPixel(point, camera_model, image_model, &pixel)) {
      continue;
    }

    projected_points.push_back({pixel, point.intensity});
    min_intensity = std::min(min_intensity, point.intensity);
    max_intensity = std::max(max_intensity, point.intensity);
  }

  const bool degenerate_range =
      projected_points.empty() || min_intensity == max_intensity;
  for (const auto& projected_point : projected_points) {
    double normalized_intensity = 0.5;
    if (!degenerate_range) {
      normalized_intensity =
          static_cast<double>(projected_point.intensity - min_intensity) /
          static_cast<double>(max_intensity - min_intensity);
    }
    cv::circle(rendered_image, projected_point.pixel, config.point_radius_px,
               PseudoColor(normalized_intensity), cv::FILLED, cv::LINE_AA);
  }

  *valid_projected_count = static_cast<int>(projected_points.size());
  *output_image = std::move(rendered_image);
  return true;
}

}  // namespace segment_projection::projection
