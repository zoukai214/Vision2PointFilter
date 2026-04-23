#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace segment_projection::pose_parse {

struct InspvaxSample {
  int64_t timestamp_ms = 0;
  Eigen::Vector3d utm_m = Eigen::Vector3d::Zero();
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  double altitude_m = 0.0;
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double azimuth_deg = 0.0;
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  int utm_zone = 0;
  bool utm_northp = true;
};

class InspvaxAscParser {
 public:
  struct Options {
    bool apply_utm_convergence = true;
  };

  struct Summary {
    int64_t parsed = 0;
    int64_t skipped = 0;
  };

  void SetOptions(const Options& options) { options_ = options; }
  const Options& options() const { return options_; }

  bool ParseFile(const std::string& asc_path, Summary& summary);
  const std::vector<InspvaxSample>& Samples() const { return samples_; }

 private:
  struct UtmResult {
    Eigen::Vector3d utm_m = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    int zone = 0;
    bool northp = true;
    double convergence_rad = 0.0;
  };

  bool ParseLine(const std::string& raw_line, InspvaxSample* sample,
                 std::string* error);
  bool ToUtmPose(double lat_deg, double lon_deg, double alt_m, double roll_deg,
                 double pitch_deg, double azimuth_deg, UtmResult* result,
                 std::string* error) const;
  static int ComputeUtmZone(double lon_deg);
  static double ConvergenceRad(double lat_deg, double lon_deg);

  Options options_;
  std::vector<InspvaxSample> samples_;
};

}  // namespace segment_projection::pose_parse
