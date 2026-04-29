#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>

#include "data_loader/gac_pcd_point.h"
#include "projection/front_wide_projection_types.h"
#include "projection/semantic_label_mapping.h"

namespace segment_projection::projection {

struct SemanticLookupContext {
  const CameraModel* camera_model = nullptr;
  const cv::Mat* semantic_image = nullptr;
  const SemanticLabelMapping* mapping = nullptr;
};

bool LookupSemanticLabelForPoint(
    const segment_projection::data_loader::GacPcdPoint& point,
    const CameraModel& camera_model, const cv::Mat& semantic_image,
    const SemanticLabelMapping& mapping, int* semantic_label);

bool LookupSemanticLabelForPointMultiCamera(
    const segment_projection::data_loader::GacPcdPoint& point,
    const std::vector<SemanticLookupContext>& lookup_contexts,
    int* semantic_label);

}  // namespace segment_projection::projection
