#pragma once

#include <cstdint>

namespace segment_projection::common::time {

double GetGPSEpochSecond(double gps_week, double gps_second);

}  // namespace segment_projection::common::time
