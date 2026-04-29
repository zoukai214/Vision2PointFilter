#pragma once

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>

namespace segment_projection::data_loader {

struct SemanticLabeledPoint {
  PCL_ADD_POINT4D;
  int intensity;
  int ring;
  int point_time_offset;
  int semantic_label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace segment_projection::data_loader

POINT_CLOUD_REGISTER_POINT_STRUCT(
    segment_projection::data_loader::SemanticLabeledPoint,
    (float, x, x)(float, y, y)(float, z, z)(int, intensity, intensity)(
        int, ring, ring)(int, point_time_offset, point_time_offset)(
        int, semantic_label, semantic_label))
