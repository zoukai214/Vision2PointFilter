#include "common/time/time_related.h"

namespace segment_projection::common::time {

double GetGPSEpochSecond(double gps_week, double gps_second) {
  return 315964800.0 + gps_week * 7.0 * 24.0 * 60.0 * 60.0 + gps_second - 18.0;
}

}  // namespace segment_projection::common::time
