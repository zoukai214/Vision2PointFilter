#include <filesystem>
#include <fstream>
#include <iostream>
#include <system_error>
#include <string>

#include "application/deskew_clip_export_config.h"
#include "data_loader/gac_clip_root_loader.h"

namespace {

bool WriteTextFile(const std::filesystem::path& path,
                   const std::string& content) {
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
      "deskew_clip_export_config_test";
  std::error_code ec;
  std::filesystem::remove_all(temp_dir, ec);
  std::filesystem::create_directories(temp_dir, ec);
  if (ec) {
    std::cerr << "failed to create temp dir: " << ec.message() << "\n";
    return 1;
  }

  const std::filesystem::path config_path = temp_dir / "config.yaml";
  {
    std::ofstream ofs(config_path);
    ofs << "deskew_clip_export:\n";
    ofs << "  projection:\n";
    ofs << "    enabled: true\n";
    ofs << "    image_root_subdir: images_seg_mask2former\n";
    ofs << "    camera_names: [front_wide, back, left_front]\n";
    ofs << "    output_subdir: projection\n";
    ofs << "    max_time_diff_ms: 120.0\n";
    ofs << "    point_radius_px: 3\n";
    ofs << "    intensity_color_map: turbo\n";
  }

  segment_projection::application::DeskewClipExportConfig cfg;
  std::string error;
  if (!segment_projection::application::LoadDeskewClipExportConfig(
          config_path, &cfg, &error)) {
    std::cerr << "LoadDeskewClipExportConfig failed: " << error << "\n";
    return 1;
  }
  if (cfg.projection.camera_names.size() != 3 ||
      cfg.projection.camera_names[1] != "back") {
    std::cerr << "unexpected camera_names order\n";
    return 1;
  }
  if (cfg.projection.image_root_subdir != "images_seg_mask2former") {
    std::cerr << "unexpected image_root_subdir\n";
    return 1;
  }

  const std::filesystem::path disabled_path = temp_dir / "disabled.yaml";
  {
    std::ofstream ofs(disabled_path);
    ofs << "deskew_clip_export:\n";
    ofs << "  projection:\n";
    ofs << "    enabled: false\n";
  }
  if (!segment_projection::application::LoadDeskewClipExportConfig(
          disabled_path, &cfg, &error)) {
    std::cerr << "disabled projection config should load: " << error << "\n";
    return 1;
  }
  if (cfg.projection.enabled || !cfg.projection.camera_names.empty()) {
    std::cerr << "disabled projection should skip strict camera validation\n";
    return 1;
  }

  const std::filesystem::path missing_projection_path =
      temp_dir / "missing_projection.yaml";
  {
    std::ofstream ofs(missing_projection_path);
    ofs << "deskew_clip_export:\n";
    ofs << "  frame_stride: 1\n";
  }
  if (segment_projection::application::LoadDeskewClipExportConfig(
          missing_projection_path, &cfg, &error)) {
    std::cerr << "missing projection block should fail when enabled defaults to true\n";
    return 1;
  }

  const std::filesystem::path duplicate_path = temp_dir / "duplicate.yaml";
  {
    std::ofstream ofs(duplicate_path);
    ofs << "deskew_clip_export:\n";
    ofs << "  projection:\n";
    ofs << "    enabled: true\n";
    ofs << "    image_root_subdir: images_seg_mask2former\n";
    ofs << "    camera_names: [front_wide, front_wide]\n";
    ofs << "    output_subdir: projection\n";
    ofs << "    max_time_diff_ms: 100.0\n";
    ofs << "    point_radius_px: 2\n";
    ofs << "    intensity_color_map: turbo\n";
  }
  if (segment_projection::application::LoadDeskewClipExportConfig(
          duplicate_path, &cfg, &error)) {
    std::cerr << "duplicate camera_names should fail\n";
    return 1;
  }

  segment_projection::data_loader::GacClipRootLoader clip(temp_dir / "clip");
  const auto image_dir =
      clip.CameraImageDir("images_seg_mask2former", "left_front");
  const auto calib_path = clip.CameraToCarPath("left_front");
  if (image_dir.filename() != "left_front") {
    std::cerr << "unexpected image_dir: " << image_dir << "\n";
    return 1;
  }
  if (calib_path.filename() != "calib_camera_left_front_to_car.json") {
    std::cerr << "unexpected calib_path: " << calib_path << "\n";
    return 1;
  }
  return 0;
}
