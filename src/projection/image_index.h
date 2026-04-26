#pragma once

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <glog/logging.h>

#include "projection/front_wide_projection_types.h"

namespace segment_projection::projection {

class ImageIndex {
 public:
  static std::unique_ptr<ImageIndex> Build(
      const std::filesystem::path& image_dir) {
    if (!std::filesystem::exists(image_dir)) {
      LOG(ERROR) << "Image directory does not exist: " << image_dir;
      return nullptr;
    }

    std::vector<ImageMatch> entries;
    for (const auto& entry : std::filesystem::directory_iterator(image_dir)) {
      if (!entry.is_regular_file() || entry.path().extension() != ".png") {
        continue;
      }
      const std::string stem = entry.path().stem().string();
      const std::size_t underscore_pos = stem.find('_');
      if (underscore_pos == std::string::npos) {
        continue;
      }

      try {
        const int64_t timestamp_ns =
            static_cast<int64_t>(std::stoll(stem.substr(0, underscore_pos)));
        entries.push_back(ImageMatch{
            timestamp_ns / 1000000,
            0,
            std::filesystem::absolute(entry.path()),
        });
      } catch (const std::exception&) {
        LOG(WARNING) << "Failed to parse timestamp from image filename: "
                     << stem;
      }
    }

    std::sort(entries.begin(), entries.end(),
              [](const ImageMatch& lhs, const ImageMatch& rhs) {
                if (lhs.timestamp_ms != rhs.timestamp_ms) {
                  return lhs.timestamp_ms < rhs.timestamp_ms;
                }
                return lhs.path < rhs.path;
              });

    return std::unique_ptr<ImageIndex>(
        new ImageIndex(image_dir, std::move(entries)));
  }

  std::optional<ImageMatch> FindNearestOrNull(int64_t target_timestamp_ms) const {
    if (entries_.empty()) {
      return std::nullopt;
    }

    const auto lower = std::lower_bound(
        entries_.begin(), entries_.end(), target_timestamp_ms,
        [](const ImageMatch& entry, int64_t target) {
          return entry.timestamp_ms < target;
        });

    const ImageMatch* best = nullptr;
    int64_t best_delta = std::numeric_limits<int64_t>::max();
    if (lower != entries_.end()) {
      best = &(*lower);
      best_delta = AbsDiff(lower->timestamp_ms, target_timestamp_ms);
    }
    if (lower != entries_.begin()) {
      const auto prev = std::prev(lower);
      const int64_t prev_delta =
          AbsDiff(prev->timestamp_ms, target_timestamp_ms);
      if (!best || prev_delta <= best_delta) {
        best = &(*prev);
        best_delta = prev_delta;
      }
    }
    if (!best) {
      return std::nullopt;
    }

    ImageMatch match = *best;
    match.delta_ms = best_delta;
    return match;
  }

  const std::filesystem::path& image_dir() const { return image_dir_; }
  const std::vector<ImageMatch>& entries() const { return entries_; }

 private:
  ImageIndex(std::filesystem::path image_dir, std::vector<ImageMatch> entries)
      : image_dir_(std::move(image_dir)), entries_(std::move(entries)) {}

  static int64_t AbsDiff(int64_t lhs, int64_t rhs) {
    return lhs >= rhs ? lhs - rhs : rhs - lhs;
  }

  std::filesystem::path image_dir_;
  std::vector<ImageMatch> entries_;
};

}  // namespace segment_projection::projection
