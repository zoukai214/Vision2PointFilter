#include "projection/camera_calibration.h"

#include <cmath>
#include <filesystem>
#include <iostream>

int main() {
  const std::filesystem::path clip_root = "/workspace/GACRT026_1758521322";
  const auto calib_dir = clip_root / "calib_extract";

  segment_projection::projection::FrontWideCameraModel model;
  const bool ok = segment_projection::projection::LoadFrontWideCameraModel(
      calib_dir / "calib_lidar_top_to_car.json",
      calib_dir / "calib_camera_front_wide_to_car.json", &model);
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
  if (!model.T_lidar_car.isApprox(model.T_lidar_car.inverse().inverse(),
                                  1e-12)) {
    std::cerr << "T_lidar_car failed basic consistency check\n";
    return 1;
  }
  if (!model.T_cam_car.isApprox(model.T_car_cam.inverse(), 1e-9)) {
    std::cerr << "expected T_cam_car to equal inverse(T_car_cam)\n";
    return 1;
  }

  return 0;
}
