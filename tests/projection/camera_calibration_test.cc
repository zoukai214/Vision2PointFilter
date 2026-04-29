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
  const auto left_front_camera_path =
      temp_dir / "calib_camera_left_front_to_car.json";
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
  "camera-front-wide": {
    "param": {
      "cam_matrix": {
        "data": [
          [1344.4, 0.0, 1910.5],
          [0.0, 1344.4, 1080.25],
          [0.0, 0.0, 1.0]
        ]
      },
      "width": 3840,
      "height": 2160,
      "distortion": {
        "data": [0.1, -0.2, 0.0, 0.0, 0.0]
      }
    }
  },
  "camera-front-wide-to-car": {
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
  },
  "camera-front-wide-undistort": {
    "param": {
      "cam_matrix": {
        "data": [
          [1.0, 0.0, 1.0],
          [0.0, 1.0, 1.0],
          [0.0, 0.0, 1.0]
        ]
      }
    }
  }
})json")) {
    std::cerr << "failed to write fixture files\n";
    return 1;
  }

  segment_projection::projection::CameraModel front_model;
  if (!segment_projection::projection::LoadCameraModel(
          "front_wide", lidar_path, camera_path, &front_model)) {
    std::cerr << "LoadCameraModel(front_wide) failed\n";
    return 1;
  }
  if (front_model.camera_name != "front_wide") {
    std::cerr << "unexpected camera_name\n";
    return 1;
  }

  if (!WriteTextFile(
          left_front_camera_path,
          R"json({
  "camera-left-front": {
    "param": {
      "cam_matrix": {
        "data": [
          [900.0, 0.0, 960.0],
          [0.0, 900.0, 540.0],
          [0.0, 0.0, 1.0]
        ]
      },
      "width": 1920,
      "height": 1080
    }
  },
  "camera-left-front-to-car": {
    "param": {
      "sensor_calib": {
        "data": [
          [1.0, 0.0, 0.0, -1.0],
          [0.0, 1.0, 0.0, 0.5],
          [0.0, 0.0, 1.0, 1.0],
          [0.0, 0.0, 0.0, 1.0]
        ]
      }
    }
  }
})json")) {
    std::cerr << "failed to write left_front fixture\n";
    return 1;
  }

  segment_projection::projection::CameraModel left_front_model;
  if (!segment_projection::projection::LoadCameraModel(
          "left_front", lidar_path, left_front_camera_path,
          &left_front_model)) {
    std::cerr << "LoadCameraModel(left_front) failed\n";
    return 1;
  }
  if (left_front_model.image_width != 1920) {
    std::cerr << "unexpected left_front width\n";
    return 1;
  }

  return 0;
}
