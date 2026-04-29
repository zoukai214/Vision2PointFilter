#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace segment_projection::data_loader {

class GacClipRootLoader {
 public:
  struct PoseSample {
    int64_t timestamp_ms = 0;
    Eigen::Vector3d utm_m = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  };

  explicit GacClipRootLoader(std::filesystem::path clip_root);

  void SetOutputDir(std::filesystem::path output_dir);

  const std::filesystem::path& clip_root() const { return clip_root_; }
  const std::filesystem::path& output_dir() const { return output_dir_; }

  std::filesystem::path CalibExtractDir() const;
  std::filesystem::path EgoRawDir() const;
  std::filesystem::path LidarRawTopDir() const;

  std::filesystem::path GnssToLidarTopEnuPath() const;
  std::filesystem::path LidarTopToCarPath() const;
  std::filesystem::path CameraFrontWideToCarPath() const;
  std::filesystem::path CameraToCarPath(const std::string& camera_name) const;
  std::filesystem::path IePostTrajAscPath() const;
  std::filesystem::path FrontWideImageDir(
      const std::string& image_subdir) const;
  std::filesystem::path CameraImageDir(
      const std::string& image_root_subdir,
      const std::string& camera_name) const;

  static Eigen::Matrix4d InvertRigidTransform(const Eigen::Matrix4d& T);
  static bool LoadCalibMatrix(const std::filesystem::path& calib_path,
                              const std::vector<std::string>& keys,
                              Eigen::Matrix4d* T_out);
  static bool LoadGnssToLidarTop(const std::filesystem::path& calib_path,
                                 Eigen::Matrix4d* T_gnss_lidar);
  static bool LoadGnssToLidarTopEnu(const std::filesystem::path& calib_path,
                                    Eigen::Matrix4d* T_gnss_lidar);
  static std::map<int64_t, std::filesystem::path> GetPcdFilesWithTimestampsMs(
      const std::filesystem::path& lidar_dir);

 private:
  std::filesystem::path clip_root_;
  std::filesystem::path output_dir_;
};

}  // namespace segment_projection::data_loader
