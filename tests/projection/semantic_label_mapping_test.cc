#include "projection/semantic_label_mapping.h"

#include <filesystem>
#include <fstream>
#include <iostream>

namespace {

std::filesystem::path WriteFile(const std::filesystem::path& dir,
                                const std::string& name,
                                const std::string& content) {
  const auto path = dir / name;
  std::ofstream(path) << content;
  return path;
}

}  // namespace

int main() {
  const auto temp_dir =
      std::filesystem::temp_directory_path() / "semantic_label_mapping_test";
  std::error_code ec;
  std::filesystem::remove_all(temp_dir, ec);
  std::filesystem::create_directories(temp_dir, ec);
  if (ec) {
    std::cerr << "failed to create temp dir: " << ec.message() << "\n";
    return 1;
  }

  const auto valid_json = WriteFile(
      temp_dir, "valid.json",
      R"json({
  "mapping": [
    {"gray_value": 0, "contiguous_id": 10, "name": "Bird"},
    {"gray_value": 51, "contiguous_id": 13, "name": "Road"}
  ]
})json");

  segment_projection::projection::SemanticLabelMapping mapping;
  if (!mapping.LoadFromJson(valid_json)) {
    std::cerr << "LoadFromJson(valid_json) returned false\n";
    return 1;
  }

  int semantic_label = -99;
  if (!mapping.Lookup(0, &semantic_label) || semantic_label != 10) {
    std::cerr << "gray_value 0 lookup mismatch\n";
    return 1;
  }
  if (!mapping.Lookup(51, &semantic_label) || semantic_label != 13) {
    std::cerr << "gray_value 51 lookup mismatch\n";
    return 1;
  }
  if (mapping.Lookup(255, &semantic_label)) {
    std::cerr << "unknown gray_value should not resolve\n";
    return 1;
  }

  const auto duplicate_json = WriteFile(
      temp_dir, "duplicate.json",
      R"json({
  "mapping": [
    {"gray_value": 7, "contiguous_id": 2, "name": "Curb"},
    {"gray_value": 7, "contiguous_id": 3, "name": "Fence"}
  ]
})json");
  if (mapping.LoadFromJson(duplicate_json)) {
    std::cerr << "duplicate gray_value should fail\n";
    return 1;
  }

  const auto invalid_json =
      WriteFile(temp_dir, "invalid.json", R"json({"mapping": [{}]})json");
  if (mapping.LoadFromJson(invalid_json)) {
    std::cerr << "missing fields should fail\n";
    return 1;
  }

  return 0;
}
