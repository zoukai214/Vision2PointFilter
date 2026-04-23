#include "pose_parse/inspvax_asc_parser.h"

#include <algorithm>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>

#include <gdal.h>
#include <ogr_spatialref.h>
#include <ogrsf_frmts.h>

#include <glog/logging.h>

#include "common/math/eigen_related.h"
#include "common/math/math.h"
#include "common/time/time_related.h"
#include "common/utils/file.h"

namespace segment_projection::pose_parse {
namespace {

std::string Trim(const std::string& value) {
  const std::string whitespace = " \t\r\n";
  const auto begin = value.find_first_not_of(whitespace);
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(whitespace);
  return value.substr(begin, end - begin + 1);
}

struct OgrCtDeleter {
  void operator()(OGRCoordinateTransformation* ct) const {
    if (ct) {
      OCTDestroyCoordinateTransformation(ct);
    }
  }
};

struct CachedTransform {
  OGRSpatialReference source;
  OGRSpatialReference target;
  std::unique_ptr<OGRCoordinateTransformation, OgrCtDeleter> ct;
};

OGRCoordinateTransformation* GetUtmTransform(int utm_zone) {
  static std::unordered_map<int, std::unique_ptr<CachedTransform>> cache;
  auto it = cache.find(utm_zone);
  if (it != cache.end()) {
    return it->second->ct.get();
  }

  auto entry = std::make_unique<CachedTransform>();
  entry->source.importFromEPSG(4326);
  entry->target.importFromEPSG(32600 + utm_zone);
  entry->ct.reset(OGRCreateCoordinateTransformation(&entry->source,
                                                    &entry->target));
  OGRCoordinateTransformation* ct = entry->ct.get();
  cache.emplace(utm_zone, std::move(entry));
  return ct;
}

}  // namespace

bool InspvaxAscParser::ParseFile(const std::string& asc_path, Summary& summary) {
  samples_.clear();

  std::ifstream ifs(asc_path);
  if (!ifs.is_open()) {
    LOG(ERROR) << "Failed to open ASC file: " << asc_path;
    return false;
  }

  summary = Summary{};
  std::string line;
  while (std::getline(ifs, line)) {
    InspvaxSample sample;
    std::string error;
    if (!ParseLine(line, &sample, &error)) {
      ++summary.skipped;
      if (!error.empty()) {
        LOG(WARNING) << "Skip ASC line: " << error;
      }
      continue;
    }
    samples_.push_back(sample);
    ++summary.parsed;
  }

  std::sort(samples_.begin(), samples_.end(),
            [](const InspvaxSample& a, const InspvaxSample& b) {
              return a.timestamp_ms < b.timestamp_ms;
            });
  return !samples_.empty();
}

bool InspvaxAscParser::ParseLine(const std::string& raw_line,
                                 InspvaxSample* sample,
                                 std::string* error) {
  if (!sample) {
    return false;
  }
  const std::string line = Trim(raw_line);
  if (line.empty()) {
    return false;
  }

  const auto semicolon_pos = line.find(';');
  if (semicolon_pos == std::string::npos) {
    if (error) {
      *error = "missing ';' separator";
    }
    return false;
  }

  const auto header_tokens =
      segment_projection::common::StringSplit(line.substr(0, semicolon_pos), ",");
  if (header_tokens.size() < 7) {
    if (error) {
      *error = "insufficient header tokens";
    }
    return false;
  }

  std::string payload = line.substr(semicolon_pos + 1);
  const auto star_pos = payload.find('*');
  if (star_pos != std::string::npos) {
    payload = payload.substr(0, star_pos);
  }

  const auto tokens = segment_projection::common::StringSplit(payload, ",");
  if (tokens.size() < 23) {
    if (error) {
      *error = "insufficient payload tokens";
    }
    return false;
  }

  try {
    const double gps_week = std::stod(Trim(header_tokens[5]));
    const double gps_second = std::stod(Trim(header_tokens[6]));
    sample->timestamp_ms = static_cast<int64_t>(
        segment_projection::common::time::GetGPSEpochSecond(gps_week, gps_second) *
        1e3);

    sample->latitude_deg = std::stod(Trim(tokens[2]));
    sample->longitude_deg = std::stod(Trim(tokens[3]));
    sample->altitude_m = std::stod(Trim(tokens[4]));
    sample->roll_deg = std::stod(Trim(tokens[9]));
    sample->pitch_deg = std::stod(Trim(tokens[10]));
    sample->azimuth_deg = std::stod(Trim(tokens[11]));
  } catch (const std::exception& ex) {
    if (error) {
      *error = std::string("parse failed: ") + ex.what();
    }
    return false;
  }

  UtmResult pose;
  if (!ToUtmPose(sample->latitude_deg, sample->longitude_deg, sample->altitude_m,
                 sample->roll_deg, sample->pitch_deg, sample->azimuth_deg, &pose,
                 error)) {
    return false;
  }
  sample->utm_m = pose.utm_m;
  sample->orientation = pose.orientation;
  sample->utm_zone = pose.zone;
  sample->utm_northp = pose.northp;
  return true;
}

bool InspvaxAscParser::ToUtmPose(double lat_deg, double lon_deg, double alt_m,
                                 double roll_deg, double pitch_deg,
                                 double azimuth_deg, UtmResult* result,
                                 std::string* error) const {
  if (!result) {
    return false;
  }

  double x = lat_deg;
  double y = lon_deg;
  const int zone = ComputeUtmZone(lon_deg);
  OGRCoordinateTransformation* ct = GetUtmTransform(zone);
  if (!ct || !ct->Transform(1, &x, &y)) {
    if (error) {
      *error = "UTM transformation failed";
    }
    return false;
  }

  Eigen::Vector3d enu_euler = Eigen::Vector3d::Zero();
  enu_euler.x() = segment_projection::common::math::Degree2Radian(roll_deg);
  enu_euler.y() = -segment_projection::common::math::Degree2Radian(pitch_deg);
  enu_euler.z() = -segment_projection::common::math::Degree2Radian(azimuth_deg);
  if (options_.apply_utm_convergence) {
    enu_euler.z() += ConvergenceRad(lat_deg, lon_deg);
  }

  const Eigen::Matrix3d rot =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      segment_projection::common::math::ToRotationMatrix(enu_euler);

  result->utm_m = Eigen::Vector3d(x, y, alt_m);
  result->orientation = Eigen::Quaterniond(rot).normalized();
  result->zone = zone;
  result->northp = true;
  result->convergence_rad = ConvergenceRad(lat_deg, lon_deg);
  return true;
}

int InspvaxAscParser::ComputeUtmZone(double lon_deg) {
  const int zone = static_cast<int>(std::floor((lon_deg + 180.0) / 6.0)) + 1;
  return std::clamp(zone, 1, 60);
}

double InspvaxAscParser::ConvergenceRad(double lat_deg, double lon_deg) {
  const double lon0_deg = (ComputeUtmZone(lon_deg) - 1) * 6.0 - 180.0 + 3.0;
  const double lat_rad = segment_projection::common::math::Degree2Radian(lat_deg);
  const double delta_lon_rad =
      segment_projection::common::math::Degree2Radian(lon_deg - lon0_deg);
  return std::atan(std::tan(delta_lon_rad) * std::sin(lat_rad));
}

}  // namespace segment_projection::pose_parse
