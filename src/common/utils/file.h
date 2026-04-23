#pragma once

#include <string>
#include <vector>

namespace segment_projection::common {

bool PathExists(const std::string& path);
std::vector<std::string> StringSplit(const std::string& value,
                                     const std::string& delimiter);

}  // namespace segment_projection::common
