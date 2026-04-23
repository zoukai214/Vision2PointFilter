#pragma once

#include <cmath>

namespace segment_projection::common::math {

template <typename T>
inline T Degree2Radian(T value) {
  return value * M_PI / 180.0;
}

}  // namespace segment_projection::common::math
