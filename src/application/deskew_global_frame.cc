#include "application/deskew_global_frame.h"

#include <algorithm>

#include <glog/logging.h>

namespace segment_projection::application {
namespace {

const pose_parse::InspvaxSample* FindSampleBefore(
    const std::vector<pose_parse::InspvaxSample>& samples, int64_t target_ms) {
  if (samples.empty()) {
    return nullptr;
  }

  auto it = std::upper_bound(
      samples.begin(), samples.end(), target_ms,
      [](int64_t value, const pose_parse::InspvaxSample& sample) {
        return value < sample.timestamp_ms;
      });
  if (it == samples.begin()) {
    return nullptr;
  }
  --it;
  return &(*it);
}

int FindSampleIndex(const std::vector<pose_parse::InspvaxSample>& samples,
                    const pose_parse::InspvaxSample* sample) {
  if (!sample || samples.empty()) {
    return -1;
  }
  const auto* begin = samples.data();
  const auto* end = begin + samples.size();
  if (sample < begin || sample >= end) {
    return -1;
  }
  return static_cast<int>(sample - begin);
}

bool ConvertSamplesToLocalEnu(
    const pose_parse::InspvaxAscParser& parser,
    const Eigen::Vector3d& origin_llh_deg_m,
    std::vector<pose_parse::InspvaxSample>* samples, std::string* error) {
  if (!samples) {
    if (error) {
      *error = "samples is null";
    }
    return false;
  }
  for (auto& sample : *samples) {
    Eigen::Vector3d enu_m = Eigen::Vector3d::Zero();
    std::string local_error;
    if (!parser.ToLocalEnu(sample.latitude_deg, sample.longitude_deg,
                           sample.altitude_m, origin_llh_deg_m, &enu_m,
                           &local_error)) {
      if (error) {
        *error = local_error;
      }
      return false;
    }
    sample.utm_m = enu_m;
  }
  return true;
}

void RecenterSamplePositions(
    const Eigen::Vector3d& origin_xyz,
    std::vector<pose_parse::InspvaxSample>* samples) {
  if (!samples) {
    return;
  }
  for (auto& sample : *samples) {
    sample.utm_m -= origin_xyz;
  }
}

}  // namespace

const char* GlobalFrameName(GlobalFrame frame) {
  switch (frame) {
    case GlobalFrame::kEnu:
      return "enu";
    case GlobalFrame::kUtm:
      return "utm";
  }
  return "unknown";
}

bool ParseGlobalFrameString(const std::string& value, GlobalFrame* frame,
                            std::string* error) {
  if (!frame) {
    if (error) {
      *error = "frame is null";
    }
    return false;
  }
  if (value == "enu") {
    *frame = GlobalFrame::kEnu;
    return true;
  }
  if (value == "utm") {
    *frame = GlobalFrame::kUtm;
    return true;
  }
  if (error) {
    *error = "pose.coord_frame must be one of: enu, utm";
  }
  return false;
}

bool ShouldApplyUtmConvergence(GlobalFrame frame) {
  return frame != GlobalFrame::kEnu;
}

std::filesystem::path SelectGnssToLidarCalibrationPath(
    const data_loader::GacClipRootLoader& clip, GlobalFrame frame) {
  static_cast<void>(frame);
  return clip.GnssToLidarTopEnuPath();
}

bool RewriteSamplesForGlobalFrame(
    const pose_parse::InspvaxAscParser& parser, GlobalFrame frame,
    int64_t origin_ts_ms, std::vector<pose_parse::InspvaxSample>* samples,
    std::string* error) {
  if (!samples) {
    if (error) {
      *error = "samples is null";
    }
    return false;
  }
  if (frame == GlobalFrame::kUtm) {
    return true;
  }
  if (samples->empty()) {
    if (error) {
      *error = "samples is empty";
    }
    return false;
  }

  const pose_parse::InspvaxSample* origin_sample =
      FindSampleBefore(*samples, origin_ts_ms);
  if (!origin_sample) {
    origin_sample = &samples->front();
    LOG(WARNING) << "No GNSS sample before origin timestamp " << origin_ts_ms
                 << ", using first sample for ENU origin.";
  }
  const int origin_index = FindSampleIndex(*samples, origin_sample);
  if (origin_index < 0) {
    if (error) {
      *error = "failed to resolve ENU origin sample";
    }
    return false;
  }

  const Eigen::Vector3d origin_llh_deg_m(origin_sample->latitude_deg,
                                         origin_sample->longitude_deg,
                                         origin_sample->altitude_m);
  if (!ConvertSamplesToLocalEnu(parser, origin_llh_deg_m, samples, error)) {
    return false;
  }
  RecenterSamplePositions((*samples)[origin_index].utm_m, samples);
  return true;
}

}  // namespace segment_projection::application
