#include "application/deskew_global_frame.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "data_loader/gac_clip_root_loader.h"
#include "pose_parse/inspvax_asc_parser.h"

namespace {

bool WriteTextFile(const std::filesystem::path& path, const std::string& content) {
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    return false;
  }
  ofs << content;
  return ofs.good();
}

std::filesystem::path WriteAscFixture() {
  const std::filesystem::path temp_dir =
      std::filesystem::temp_directory_path() /
      std::filesystem::path("deskew_global_frame_test_fixture");
  std::error_code ec;
  std::filesystem::remove_all(temp_dir, ec);
  std::filesystem::create_directories(temp_dir, ec);
  if (ec) {
    return {};
  }

  const std::filesystem::path asc_path = temp_dir / "IE_post_traj.asc";
  const std::string content =
      "INSPVAXA,COM1,0,64.0,FINESTEERING,2300,100.0;"
      "INS_SOLUTION_GOOD,INS_RTKFIXED,30.000000,120.000000,10.0,0,0,0,0,1.0,2.0,3.0,0,0,0,0,0,0,0,0,0,0,0*00\n"
      "INSPVAXA,COM1,0,64.0,FINESTEERING,2300,100.1;"
      "INS_SOLUTION_GOOD,INS_RTKFIXED,30.000010,120.000020,10.5,0,0,0,0,1.5,2.5,3.5,0,0,0,0,0,0,0,0,0,0,0*00\n";
  if (!WriteTextFile(asc_path, content)) {
    return {};
  }
  return asc_path;
}

}  // namespace

int main() {
  using segment_projection::application::GlobalFrame;

  GlobalFrame frame = GlobalFrame::kEnu;
  std::string error;
  if (!segment_projection::application::ParseGlobalFrameString("enu", &frame,
                                                               &error) ||
      frame != GlobalFrame::kEnu) {
    std::cerr << "failed to parse enu global frame\n";
    return 1;
  }
  if (!segment_projection::application::ParseGlobalFrameString("utm", &frame,
                                                               &error) ||
      frame != GlobalFrame::kUtm) {
    std::cerr << "failed to parse utm global frame\n";
    return 1;
  }
  if (segment_projection::application::ParseGlobalFrameString("bad", &frame,
                                                              &error)) {
    std::cerr << "expected invalid global frame to fail\n";
    return 1;
  }

  const std::filesystem::path clip_root = "/tmp/fake_clip_root";
  segment_projection::data_loader::GacClipRootLoader clip(clip_root);
  if (segment_projection::application::SelectGnssToLidarCalibrationPath(
          clip, GlobalFrame::kEnu) !=
      (clip_root / "calib_extract" / "calib_gnss_to_lidar_top_ENU.json")) {
    std::cerr << "unexpected enu calib path\n";
    return 1;
  }
  if (segment_projection::application::SelectGnssToLidarCalibrationPath(
          clip, GlobalFrame::kUtm) !=
      (clip_root / "calib_extract" / "calib_gnss_to_lidar_top_ENU.json")) {
    std::cerr << "unexpected utm calib path\n";
    return 1;
  }
  if (segment_projection::application::ShouldApplyUtmConvergence(
          GlobalFrame::kEnu)) {
    std::cerr << "enu should disable utm convergence\n";
    return 1;
  }
  if (!segment_projection::application::ShouldApplyUtmConvergence(
          GlobalFrame::kUtm)) {
    std::cerr << "utm should enable utm convergence\n";
    return 1;
  }

  const std::filesystem::path asc_path = WriteAscFixture();
  if (asc_path.empty()) {
    std::cerr << "failed to write ASC fixture\n";
    return 1;
  }

  segment_projection::pose_parse::InspvaxAscParser parser;
  segment_projection::pose_parse::InspvaxAscParser::Options options;
  options.apply_utm_convergence = false;
  parser.SetOptions(options);
  segment_projection::pose_parse::InspvaxAscParser::Summary summary;
  if (!parser.ParseFile(asc_path.string(), summary)) {
    std::cerr << "failed to parse ASC fixture\n";
    return 1;
  }
  std::vector<segment_projection::pose_parse::InspvaxSample> samples =
      parser.Samples();
  if (samples.size() != 2) {
    std::cerr << "unexpected sample count\n";
    return 1;
  }
  const Eigen::Vector3d original_second = samples[1].utm_m;
  if (!segment_projection::application::RewriteSamplesForGlobalFrame(
          parser, GlobalFrame::kEnu, samples.front().timestamp_ms + 1,
          &samples, &error)) {
    std::cerr << "failed to rewrite ENU samples: " << error << "\n";
    return 1;
  }
  if (samples.front().utm_m.norm() > 1e-4) {
    std::cerr << "expected origin ENU sample near zero, got "
              << samples.front().utm_m.transpose() << "\n";
    return 1;
  }
  if ((samples[1].utm_m - original_second).norm() < 1.0) {
    std::cerr << "expected second sample to change after ENU rewrite\n";
    return 1;
  }

  std::vector<segment_projection::pose_parse::InspvaxSample> utm_samples =
      parser.Samples();
  const Eigen::Vector3d utm_second = utm_samples[1].utm_m;
  if (!segment_projection::application::RewriteSamplesForGlobalFrame(
          parser, GlobalFrame::kUtm, utm_samples.front().timestamp_ms + 1,
          &utm_samples, &error)) {
    std::cerr << "failed to keep UTM samples: " << error << "\n";
    return 1;
  }
  if ((utm_samples[1].utm_m - utm_second).norm() > 1e-9) {
    std::cerr << "expected UTM samples to remain unchanged\n";
    return 1;
  }

  return 0;
}
