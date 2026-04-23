#include "common/utils/file.h"

#include <filesystem>

namespace segment_projection::common {

bool PathExists(const std::string& path) {
  return std::filesystem::exists(path);
}

std::vector<std::string> StringSplit(const std::string& value,
                                     const std::string& delimiter) {
  std::vector<std::string> tokens;
  if (delimiter.empty()) {
    tokens.push_back(value);
    return tokens;
  }
  std::size_t start = 0;
  while (true) {
    const std::size_t pos = value.find(delimiter, start);
    if (pos == std::string::npos) {
      tokens.push_back(value.substr(start));
      break;
    }
    tokens.push_back(value.substr(start, pos - start));
    start = pos + delimiter.size();
  }
  return tokens;
}

}  // namespace segment_projection::common
