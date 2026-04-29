#include "projection/semantic_point_labeler.h"

#include "projection/point_cloud_projector.h"

namespace segment_projection::projection {

bool LookupSemanticLabelForPoint(
    const segment_projection::data_loader::GacPcdPoint& point,
    const CameraModel& camera_model, const cv::Mat& semantic_image,
    const SemanticLabelMapping& mapping, int* semantic_label) {
  if (!semantic_label || semantic_image.empty() ||
      semantic_image.type() != CV_8UC1) {
    return false;
  }

  *semantic_label = -1;

  cv::Point pixel;
  if (!ProjectLidarPointToPixel(point, camera_model, &pixel)) {
    return true;
  }
  if (pixel.x < 0 || pixel.x >= semantic_image.cols || pixel.y < 0 ||
      pixel.y >= semantic_image.rows) {
    return true;
  }

  const auto gray_value = semantic_image.at<std::uint8_t>(pixel.y, pixel.x);
  int mapped_label = -1;
  if (!mapping.Lookup(gray_value, &mapped_label)) {
    return true;
  }

  *semantic_label = mapped_label;
  return true;
}

bool LookupSemanticLabelForPointMultiCamera(
    const segment_projection::data_loader::GacPcdPoint& point,
    const std::vector<SemanticLookupContext>& lookup_contexts,
    int* semantic_label) {
  if (!semantic_label) {
    return false;
  }

  *semantic_label = -1;
  for (const SemanticLookupContext& context : lookup_contexts) {
    if (!context.camera_model || !context.semantic_image || !context.mapping) {
      return false;
    }

    int candidate_label = -1;
    if (!LookupSemanticLabelForPoint(point, *context.camera_model,
                                     *context.semantic_image, *context.mapping,
                                     &candidate_label)) {
      return false;
    }
    if (candidate_label >= 0) {
      *semantic_label = candidate_label;
      return true;
    }
  }
  return true;
}

}  // namespace segment_projection::projection
