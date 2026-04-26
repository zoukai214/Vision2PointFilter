#include "projection/camera_calibration.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {

bool WriteTextFile(const std::filesystem::path& path, const std::string& content) {
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    return false;
  }
  ofs << content;
  return ofs.good();
}

}  // namespace

int main() {
  const std::filesystem::path temp_dir =
      std::filesystem::temp_directory_path() /
      std::filesystem::path("camera_calibration_test_fixture");
  std::error_code ec;
  std::filesystem::remove_all(temp_dir, ec);
  std::filesystem::create_directories(temp_dir, ec);
  if (ec) {
    std::cerr << "failed to create temp dir: " << ec.message() << "\n";
    return 1;
  }

  const auto lidar_path = temp_dir / "calib_lidar_top_to_car.json";
  const auto camera_path = temp_dir / "calib_camera_front_wide_to_car.json";
  if (!WriteTextFile(
          lidar_path,
          R"json({
  "lidar-top-to-car": {
    "param": {
      "sensor_calib": {
        "data": [
          [1.0, 0.0, 0.0, 1.5],
          [0.0, 1.0, 0.0, -2.0],
          [0.0, 0.0, 1.0, 0.25],
          [0.0, 0.0, 0.0, 1.0]
        ]
      }
    }
  }
})json") ||
      !WriteTextFile(
          camera_path,
          R"json({
  "camera-front-wide-undistort": {
    "param": {
      "cam_matrix": {
        "data": [
          [1344.4, 0.0, 1910.5],
          [0.0, 1344.4, 1080.25],
          [0.0, 0.0, 1.0]
        ]
      },
      "width": 3840,
      "height": 2160
    }
  },
  "camera-front-wide-to-car-undistort": {
    "param": {
      "sensor_calib": {
        "data": [
          [0.0, -1.0, 0.0, 2.0],
          [1.0, 0.0, 0.0, 0.5],
          [0.0, 0.0, 1.0, 1.25],
          [0.0, 0.0, 0.0, 1.0]
        ]
      }
    }
  }
})json")) {
    std::cerr << "failed to write fixture files\n";
    return 1;
  }

  segment_projection::projection::FrontWideCameraModel model;
  const bool ok = segment_projection::projection::LoadFrontWideCameraModel(
      lidar_path, camera_path, &model);
  if (!ok) {
    std::cerr << "LoadFrontWideCameraModel returned false\n";
    return 1;
  }
  if (model.image_width != 3840 || model.image_height != 2160) {
    std::cerr << "unexpected image size: " << model.image_width << "x"
              << model.image_height << "\n";
    return 1;
  }
  if (std::abs(model.K(0, 0) - 1344.4) > 1e-6) {
    std::cerr << "unexpected undistorted fx: " << model.K(0, 0) << "\n";
    return 1;
  }
  Eigen::Matrix4d expected_T_car_lidar = Eigen::Matrix4d::Identity();
  expected_T_car_lidar(0, 3) = 1.5;
  expected_T_car_lidar(1, 3) = -2.0;
  expected_T_car_lidar(2, 3) = 0.25;
  if (!model.T_car_lidar.isApprox(expected_T_car_lidar, 1e-12)) {
    std::cerr << "unexpected T_car_lidar\n";
    return 1;
  }
  const Eigen::Matrix4d expected_T_lidar_car = expected_T_car_lidar.inverse();
  if (!model.T_lidar_car.isApprox(expected_T_lidar_car, 1e-12)) {
    std::cerr << "unexpected T_lidar_car\n";
    return 1;
  }
  if (!model.T_cam_car.isApprox(model.T_car_cam.inverse(), 1e-9)) {
    std::cerr << "expected T_cam_car to equal inverse(T_car_cam)\n";
    return 1;
  }
  Eigen::Matrix4d expected_T_car_cam = Eigen::Matrix4d::Identity();
  expected_T_car_cam(0, 0) = 0.0;
  expected_T_car_cam(0, 1) = -1.0;
  expected_T_car_cam(1, 0) = 1.0;
  expected_T_car_cam(1, 1) = 0.0;
  expected_T_car_cam(0, 3) = 2.0;
  expected_T_car_cam(1, 3) = 0.5;
  expected_T_car_cam(2, 3) = 1.25;
  if (!model.T_car_cam.isApprox(expected_T_car_cam, 1e-12)) {
    std::cerr << "unexpected T_car_cam\n";
    return 1;
  }
  if (!model.T_cam_car.isApprox(model.T_car_cam.inverse(), 1e-9)) {
    std::cerr << "expected T_cam_car to equal inverse(T_car_cam)\n";
    return 1;
  }

  if (!WriteTextFile(
          camera_path,
          R"json({
  "camera-front-wide-undistort": {
    "param": {
      "cam_matrix": {
        "data": [
          [1344.4, 0.0, 1910.5],
          [0.0, 1344.4, 1080.25],
          [0.0, 0.0, 1.0]
        ]
      },
      "width": 0,
      "height": -1
    }
  },
  "camera-front-wide-to-car-undistort": {
    "param": {
      "sensor_calib": {
        "data": [
          [0.0, -1.0, 0.0, 2.0],
          [1.0, 0.0, 0.0, 0.5],
          [0.0, 0.0, 1.0, 1.25],
          [0.0, 0.0, 0.0, 1.0]
        ]
      }
    }
  }
})json")) {
    std::cerr << "failed to write invalid camera fixture\n";
    return 1;
  }
  if (segment_projection::projection::LoadFrontWideCameraModel(
          lidar_path, camera_path, &model)) {
    std::cerr << "expected LoadFrontWideCameraModel to reject non-positive image size\n";
    return 1;
  }

  return 0;
}
