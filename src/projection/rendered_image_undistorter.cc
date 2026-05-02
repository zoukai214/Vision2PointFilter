#include "projection/rendered_image_undistorter.h"

#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace segment_projection::projection {
namespace {

bool ValidateInput(const cv::Mat& source_image,
                   const CameraModel& camera_model,
                   cv::Mat* undistorted_image) {
  if (source_image.empty() || undistorted_image == nullptr) {
    return false;
  }
  if (source_image.cols != camera_model.image_width ||
      source_image.rows != camera_model.image_height) {
    return false;
  }
  return !camera_model.distortion_coeffs.empty();
}

void BuildFisheyeMap(const cv::Mat& source_image, const CameraModel& camera_model,
                     cv::Mat* map_x, cv::Mat* map_y) {
  const int width = source_image.cols;
  const int height = source_image.rows;
  const double fx = camera_model.K(0, 0);
  const double fy = camera_model.K(1, 1);
  const double cx = camera_model.K(0, 2);
  const double cy = camera_model.K(1, 2);
  const double alpha = fx != 0.0 ? camera_model.K(0, 1) / fx : 0.0;

  map_x->create(height, width, CV_32FC1);
  map_y->create(height, width, CV_32FC1);

  const double k1 = camera_model.distortion_coeffs[0];
  const double k2 = camera_model.distortion_coeffs.size() > 1
                        ? camera_model.distortion_coeffs[1]
                        : 0.0;
  const double k3 = camera_model.distortion_coeffs.size() > 2
                        ? camera_model.distortion_coeffs[2]
                        : 0.0;
  const double k4 = camera_model.distortion_coeffs.size() > 3
                        ? camera_model.distortion_coeffs[3]
                        : 0.0;

  for (int v = 0; v < height; ++v) {
    float* map_x_row = map_x->ptr<float>(v);
    float* map_y_row = map_y->ptr<float>(v);
    const double y = (static_cast<double>(v) - cy) / fy;
    for (int u = 0; u < width; ++u) {
      const double x = (static_cast<double>(u) - cx) / fx;
      const double r = std::sqrt(x * x + y * y);
      double x_distorted = x;
      double y_distorted = y;
      if (r > 0.0) {
        const double theta = std::atan(r);
        const double theta2 = theta * theta;
        const double theta4 = theta2 * theta2;
        const double theta6 = theta4 * theta2;
        const double theta8 = theta4 * theta4;
        const double theta_d =
            theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 +
                     k4 * theta8);
        const double scale = theta_d / r;
        x_distorted = scale * x;
        y_distorted = scale * y;
      }

      map_x_row[u] = static_cast<float>(fx * (x_distorted + alpha * y_distorted) +
                                        cx);
      map_y_row[u] = static_cast<float>(fy * y_distorted + cy);
    }
  }
}

bool BuildDistorted8Map(const cv::Mat& source_image,
                        const CameraModel& camera_model, cv::Mat* map_x,
                        cv::Mat* map_y) {
  const int width = source_image.cols;
  const int height = source_image.rows;
  const double fx = camera_model.K(0, 0);
  const double fy = camera_model.K(1, 1);
  const double cx = camera_model.K(0, 2);
  const double cy = camera_model.K(1, 2);

  map_x->create(height, width, CV_32FC1);
  map_y->create(height, width, CV_32FC1);

  const double k1 = camera_model.distortion_coeffs[0];
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

  for (int v = 0; v < height; ++v) {
    float* map_x_row = map_x->ptr<float>(v);
    float* map_y_row = map_y->ptr<float>(v);
    const double y = (static_cast<double>(v) - cy) / fy;
    for (int u = 0; u < width; ++u) {
      const double x = (static_cast<double>(u) - cx) / fx;
      const double r2 = x * x + y * y;
      const double r4 = r2 * r2;
      const double r6 = r4 * r2;
      const double numerator = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
      const double denominator = 1.0 + k4 * r2 + k5 * r4 + k6 * r6;
      if (std::abs(denominator) <= 1e-12) {
        return false;
      }
      const double radial = numerator / denominator;

      const double x_distorted =
          x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
      const double y_distorted =
          y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;

      map_x_row[u] = static_cast<float>(fx * x_distorted + cx);
      map_y_row[u] = static_cast<float>(fy * y_distorted + cy);
    }
  }

  return true;
}

}  // namespace

bool UndistortRenderedImage(const cv::Mat& source_image,
                            const CameraModel& camera_model,
                            cv::Mat* undistorted_image) {
  if (!ValidateInput(source_image, camera_model, undistorted_image)) {
    return false;
  }

  cv::Mat map_x;
  cv::Mat map_y;
  switch (camera_model.raw_projection_model) {
    case DistortionModel::kFisheye4:
      BuildFisheyeMap(source_image, camera_model, &map_x, &map_y);
      break;
    case DistortionModel::kDistorted8:
      if (!BuildDistorted8Map(source_image, camera_model, &map_x, &map_y)) {
        return false;
      }
      break;
    default:
      return false;
  }

  cv::remap(source_image, *undistorted_image, map_x, map_y, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT);
  return true;
}

}  // namespace segment_projection::projection
