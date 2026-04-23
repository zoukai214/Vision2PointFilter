#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include "data_loader/gac_clip_root_loader.h"
#include "data_loader/gac_pcd_point.h"
#include "pose_parse/inspvax_asc_parser.h"

namespace {

using ClipLoader = segment_projection::data_loader::GacClipRootLoader;
using GacPcdPoint = segment_projection::data_loader::GacPcdPoint;
using InspvaxSample = segment_projection::pose_parse::InspvaxSample;

struct Config {
  int frame_stride = 1;
  double deskew_max_range_m = 100.0;
  double interp_max_gap_ms = 100.0;
  std::string output_subdir = "deskew_pcd";
};

struct InterpolatedPose {
  Eigen::Vector3d utm_m = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

std::string TimestampNameFromPcdPath(const std::filesystem::path& pcd_path,
                                     int64_t fallback_ms) {
  const std::string stem = pcd_path.stem().string();
  if (!stem.empty()) {
    return stem;
  }
  return std::to_string(fallback_ms);
}

bool LoadConfig(const std::filesystem::path& config_path, Config* cfg) {
  if (!cfg) {
    return false;
  }
  const YAML::Node root = YAML::LoadFile(config_path.string());
  const YAML::Node node = root["deskew_clip_export"];
  if (!node) {
    LOG(ERROR) << "Missing 'deskew_clip_export' root in " << config_path;
    return false;
  }

  if (node["frame_stride"]) {
    cfg->frame_stride = node["frame_stride"].as<int>();
  }
  if (const YAML::Node deskew = node["deskew"]) {
    if (deskew["max_range_m"]) {
      cfg->deskew_max_range_m = deskew["max_range_m"].as<double>();
    }
    if (deskew["interp_max_gap_ms"]) {
      cfg->interp_max_gap_ms = deskew["interp_max_gap_ms"].as<double>();
    }
  }
  if (const YAML::Node output = node["output"]) {
    if (output["subdir"]) {
      cfg->output_subdir = output["subdir"].as<std::string>();
    }
  }

  if (cfg->frame_stride <= 0) {
    LOG(ERROR) << "frame_stride must be > 0";
    return false;
  }
  if (!(cfg->interp_max_gap_ms > 0.0)) {
    LOG(ERROR) << "deskew.interp_max_gap_ms must be > 0";
    return false;
  }
  return true;
}

bool LoadPcdCloud(const std::string& pcd_path,
                  pcl::PointCloud<GacPcdPoint>* cloud) {
  if (!cloud) {
    return false;
  }
  if (pcl::io::loadPCDFile(pcd_path, *cloud) != 0) {
    LOG(ERROR) << "Failed to load PCD: " << pcd_path;
    return false;
  }
  return true;
}

const InspvaxSample* FindNearestSample(const std::vector<InspvaxSample>& samples,
                                       int64_t target_ms,
                                       int64_t max_diff_ms) {
  if (samples.empty()) {
    return nullptr;
  }

  auto it = std::lower_bound(
      samples.begin(), samples.end(), target_ms,
      [](const InspvaxSample& sample, int64_t value) {
        return sample.timestamp_ms < value;
      });

  const InspvaxSample* best = nullptr;
  int64_t best_diff = std::numeric_limits<int64_t>::max();
  if (it != samples.end()) {
    best = &(*it);
    best_diff = std::llabs(it->timestamp_ms - target_ms);
  }
  if (it != samples.begin()) {
    const auto prev = std::prev(it);
    const int64_t diff = std::llabs(prev->timestamp_ms - target_ms);
    if (diff < best_diff) {
      best = &(*prev);
      best_diff = diff;
    }
  }
  if (!best || best_diff > max_diff_ms) {
    return nullptr;
  }
  return best;
}

bool InterpolatePose(const std::vector<InspvaxSample>& samples,
                     double target_ms,
                     double max_gap_ms,
                     InterpolatedPose* pose) {
  if (!pose || samples.empty()) {
    return false;
  }

  auto it = std::lower_bound(
      samples.begin(), samples.end(), target_ms,
      [](const InspvaxSample& sample, double value) {
        return static_cast<double>(sample.timestamp_ms) < value;
      });

  if (it == samples.begin() || it == samples.end()) {
    return false;
  }

  const auto& s1 = *std::prev(it);
  const auto& s2 = *it;
  const double t1 = static_cast<double>(s1.timestamp_ms);
  const double t2 = static_cast<double>(s2.timestamp_ms);
  const double gap = t2 - t1;
  if (gap <= 0.0 || gap > max_gap_ms) {
    return false;
  }

  const double alpha = (target_ms - t1) / gap;
  pose->utm_m = s1.utm_m + alpha * (s2.utm_m - s1.utm_m);
  pose->orientation = s1.orientation.slerp(alpha, s2.orientation).normalized();
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;

  if (argc < 3 || argc > 4) {
    LOG(ERROR) << "Usage: " << argv[0]
               << " <config_yaml> <clip_root> [output_dir]";
    return EXIT_FAILURE;
  }

  const std::filesystem::path config_path = argv[1];
  const std::filesystem::path clip_root = argv[2];
  const std::filesystem::path output_dir =
      (argc >= 4) ? std::filesystem::path(argv[3]) : (clip_root / "output");

  Config cfg;
  if (!LoadConfig(config_path, &cfg)) {
    return EXIT_FAILURE;
  }

  ClipLoader clip(clip_root);
  clip.SetOutputDir(output_dir);

  const std::filesystem::path asc_path = clip.IePostTrajAscPath();
  const std::filesystem::path calib_path = clip.GnssToLidarTopEnuPath();
  const std::filesystem::path lidar_dir = clip.LidarRawTopDir();
  const std::filesystem::path output_pcd_dir = clip.output_dir() / cfg.output_subdir;

  std::error_code ec;
  std::filesystem::create_directories(output_pcd_dir, ec);
  if (ec) {
    LOG(ERROR) << "Failed to create output directory: " << output_pcd_dir
               << ", error=" << ec.message();
    return EXIT_FAILURE;
  }

  segment_projection::pose_parse::InspvaxAscParser parser;
  segment_projection::pose_parse::InspvaxAscParser::Summary summary;
  if (!parser.ParseFile(asc_path.string(), summary)) {
    LOG(ERROR) << "Failed to parse ASC: " << asc_path;
    return EXIT_FAILURE;
  }
  const auto& samples = parser.Samples();
  LOG(INFO) << "Parsed INSPVAX samples: " << samples.size()
            << ", skipped=" << summary.skipped;

  Eigen::Matrix4d T_gnss_lidar = Eigen::Matrix4d::Identity();
  if (!ClipLoader::LoadGnssToLidarTopEnu(calib_path, &T_gnss_lidar)) {
    LOG(ERROR) << "Failed to load GNSS->LiDAR extrinsic: " << calib_path;
    return EXIT_FAILURE;
  }

  const auto pcd_files = ClipLoader::GetPcdFilesWithTimestampsMs(lidar_dir);
  if (pcd_files.empty()) {
    LOG(ERROR) << "No PCD files found in " << lidar_dir;
    return EXIT_FAILURE;
  }

  const double deskew_max_r2 =
      (cfg.deskew_max_range_m > 0.0)
          ? cfg.deskew_max_range_m * cfg.deskew_max_range_m
          : std::numeric_limits<double>::infinity();

  int64_t frame_index = 0;
  int64_t exported_frames = 0;
  int64_t skipped_frames = 0;

  for (const auto& [timestamp_ms, pcd_path] : pcd_files) {
    if (cfg.frame_stride > 1 && (frame_index % cfg.frame_stride) != 0) {
      ++frame_index;
      continue;
    }
    ++frame_index;

    const auto* frame_sample = FindNearestSample(samples, timestamp_ms, 200);
    if (!frame_sample) {
      LOG(WARNING) << "Skip frame without nearby pose: " << pcd_path;
      ++skipped_frames;
      continue;
    }

    pcl::PointCloud<GacPcdPoint> cloud;
    if (!LoadPcdCloud(pcd_path.string(), &cloud)) {
      ++skipped_frames;
      continue;
    }

    Eigen::Matrix4d T_w_gnss_frame = Eigen::Matrix4d::Identity();
    T_w_gnss_frame.block<3, 3>(0, 0) =
        frame_sample->orientation.toRotationMatrix();
    T_w_gnss_frame.block<3, 1>(0, 3) = frame_sample->utm_m;
    const Eigen::Matrix4d T_w_lidar_frame = T_w_gnss_frame * T_gnss_lidar;
    const Eigen::Matrix4d T_lidar_frame_w = T_w_lidar_frame.inverse();
    const Eigen::Matrix3d R_lidar_frame_w =
        T_lidar_frame_w.block<3, 3>(0, 0);
    const Eigen::Vector3d t_lidar_frame_w =
        T_lidar_frame_w.block<3, 1>(0, 3);

    pcl::PointCloud<GacPcdPoint> deskewed_cloud;
    deskewed_cloud.reserve(cloud.size());

    for (const auto& point : cloud) {
      const double r2 =
          point.x * point.x + point.y * point.y + point.z * point.z;
      if (r2 > deskew_max_r2) {
        continue;
      }

      const double point_ts_ms =
          static_cast<double>(timestamp_ms) +
          static_cast<double>(point.point_time_offset) * 1e-3;

      InterpolatedPose pose;
      if (!InterpolatePose(samples, point_ts_ms, cfg.interp_max_gap_ms, &pose)) {
        continue;
      }

      Eigen::Matrix4d T_w_gnss = Eigen::Matrix4d::Identity();
      T_w_gnss.block<3, 3>(0, 0) = pose.orientation.toRotationMatrix();
      T_w_gnss.block<3, 1>(0, 3) = pose.utm_m;
      const Eigen::Matrix4d T_w_lidar = T_w_gnss * T_gnss_lidar;

      const Eigen::Vector3d p_lidar(point.x, point.y, point.z);
      const Eigen::Vector3d p_world =
          T_w_lidar.block<3, 3>(0, 0) * p_lidar + T_w_lidar.block<3, 1>(0, 3);
      const Eigen::Vector3d p_lidar_frame =
          R_lidar_frame_w * p_world + t_lidar_frame_w;

      GacPcdPoint out_point = point;
      out_point.x = static_cast<float>(p_lidar_frame.x());
      out_point.y = static_cast<float>(p_lidar_frame.y());
      out_point.z = static_cast<float>(p_lidar_frame.z());
      deskewed_cloud.push_back(out_point);
    }

    deskewed_cloud.width = static_cast<std::uint32_t>(deskewed_cloud.size());
    deskewed_cloud.height = 1;
    deskewed_cloud.is_dense = false;

    const std::filesystem::path output_path =
        output_pcd_dir / (TimestampNameFromPcdPath(pcd_path, timestamp_ms) + ".pcd");
    if (pcl::io::savePCDFileBinary(output_path.string(), deskewed_cloud) != 0) {
      LOG(ERROR) << "Failed to save deskewed PCD: " << output_path;
      ++skipped_frames;
      continue;
    }

    LOG(INFO) << "Deskewed " << pcd_path.filename() << " -> " << output_path
              << ", points=" << deskewed_cloud.size();
    ++exported_frames;
  }

  LOG(INFO) << "Deskew complete. exported_frames=" << exported_frames
            << ", skipped_frames=" << skipped_frames
            << ", output_dir=" << output_pcd_dir;
  return EXIT_SUCCESS;
}
