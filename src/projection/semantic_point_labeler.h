#pragma once

#include <opencv2/core/mat.hpp>

#include "data_loader/gac_pcd_point.h"
#include "projection/front_wide_projection_types.h"
#include "projection/semantic_label_mapping.h"

namespace segment_projection::projection {

bool LookupSemanticLabelForPoint(
    const segment_projection::data_loader::GacPcdPoint& point,
    const FrontWideCameraModel& camera_model, const cv::Mat& semantic_image,
    const SemanticLabelMapping& mapping, int* semantic_label);

}  // namespace segment_projection::projection
