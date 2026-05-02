#include "projection/rendered_image_undistorter.h"

#include <iostream>

#include <opencv2/core.hpp>

#include "projection/front_wide_projection_types.h"

namespace {

using segment_projection::projection::CameraModel;
using segment_projection::projection::DistortionModel;

CameraModel MakeCameraModel() {
  CameraModel camera_model;
  camera_model.camera_name = "front_wide";
  camera_model.K << 10.0, 0.0, 5.0, 0.0, 10.0, 5.0, 0.0, 0.0, 1.0;
  camera_model.image_width = 11;
  camera_model.image_height = 11;
  return camera_model;
}

}  // namespace

int main() {
  const cv::Mat source = [] {
    cv::Mat image = cv::Mat::zeros(11, 11, CV_8UC3);
    image.at<cv::Vec3b>(5, 7) = cv::Vec3b(10, 20, 30);
    return image;
  }();

  {
    CameraModel camera_model = MakeCameraModel();
    camera_model.distortion_coeffs = {0.0, 0.0, 0.0, 0.0};
    camera_model.raw_projection_model = DistortionModel::kFisheye4;

    cv::Mat undistorted;
    if (!segment_projection::projection::UndistortRenderedImage(
            source, camera_model, &undistorted)) {
      std::cerr << "expected fisheye undistortion to succeed\n";
      return 1;
    }
    if (undistorted.empty() || undistorted.rows != source.rows ||
        undistorted.cols != source.cols) {
      std::cerr << "expected fisheye undistortion output to keep image size\n";
      return 1;
    }
  }

  {
    CameraModel camera_model = MakeCameraModel();
    camera_model.distortion_coeffs = {0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    camera_model.raw_projection_model = DistortionModel::kDistorted8;

    cv::Mat undistorted;
    if (!segment_projection::projection::UndistortRenderedImage(
            source, camera_model, &undistorted)) {
      std::cerr << "expected distorted8 undistortion to succeed\n";
      return 1;
    }
    if (undistorted.empty() || undistorted.rows != source.rows ||
        undistorted.cols != source.cols) {
      std::cerr << "expected distorted8 undistortion output to keep image size\n";
      return 1;
    }
  }

  {
    CameraModel camera_model = MakeCameraModel();
    cv::Mat undistorted;
    if (segment_projection::projection::UndistortRenderedImage(
            source, camera_model, &undistorted)) {
      std::cerr << "expected missing raw distortion coefficients to be rejected\n";
      return 1;
    }
  }

  return 0;
}
