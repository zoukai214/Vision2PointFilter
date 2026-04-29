#pragma once

#include <cstdint>
#include <filesystem>
#include <unordered_map>

namespace segment_projection::projection {

class SemanticLabelMapping {
 public:
  bool LoadFromJson(const std::filesystem::path& json_path);
  bool Lookup(std::uint8_t gray_value, int* semantic_label) const;

 private:
  std::unordered_map<int, int> gray_to_contiguous_id_;
};

}  // namespace segment_projection::projection
