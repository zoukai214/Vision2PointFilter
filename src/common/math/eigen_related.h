#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace segment_projection::common::math {

Eigen::Matrix3d ToRotationMatrix(const Eigen::Vector3d& euler_angle);

}  // namespace segment_projection::common::math
