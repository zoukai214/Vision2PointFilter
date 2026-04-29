#include "projection/semantic_label_mapping.h"

#include <fstream>

#include <glog/logging.h>
#include <json/json.h>

namespace segment_projection::projection {

bool SemanticLabelMapping::LoadFromJson(
    const std::filesystem::path& json_path) {
  gray_to_contiguous_id_.clear();

  std::ifstream input(json_path);
  if (!input.is_open()) {
    LOG(ERROR) << "Failed to open semantic mapping json: " << json_path;
    return false;
  }

  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string errors;
  if (!Json::parseFromStream(builder, input, &root, &errors)) {
    LOG(ERROR) << "Failed to parse semantic mapping json: " << json_path
               << ", errors=" << errors;
    return false;
  }

  const Json::Value mapping = root["mapping"];
  if (!mapping.isArray()) {
    LOG(ERROR) << "semantic mapping json missing array field 'mapping': "
               << json_path;
    return false;
  }

  for (const auto& entry : mapping) {
    if (!entry.isMember("gray_value") || !entry.isMember("contiguous_id") ||
        !entry["gray_value"].isInt() || !entry["contiguous_id"].isInt()) {
      LOG(ERROR) << "semantic mapping entry missing gray_value/contiguous_id";
      return false;
    }

    const int gray_value = entry["gray_value"].asInt();
    const int contiguous_id = entry["contiguous_id"].asInt();
    if (gray_to_contiguous_id_.count(gray_value) != 0U) {
      LOG(ERROR) << "duplicate gray_value in semantic mapping: " << gray_value;
      return false;
    }
    gray_to_contiguous_id_.emplace(gray_value, contiguous_id);
  }

  return !gray_to_contiguous_id_.empty();
}

bool SemanticLabelMapping::Lookup(std::uint8_t gray_value,
                                  int* semantic_label) const {
  if (!semantic_label) {
    return false;
  }

  const auto it = gray_to_contiguous_id_.find(static_cast<int>(gray_value));
  if (it == gray_to_contiguous_id_.end()) {
    return false;
  }

  *semantic_label = it->second;
  return true;
}

}  // namespace segment_projection::projection
