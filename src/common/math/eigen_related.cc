#include "common/math/eigen_related.h"

namespace segment_projection::common::math {

Eigen::Matrix3d ToRotationMatrix(const Eigen::Vector3d& euler_angle) {
  return (Eigen::AngleAxisd(euler_angle.z(), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(euler_angle.y(), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(euler_angle.x(), Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

}  // namespace segment_projection::common::math
