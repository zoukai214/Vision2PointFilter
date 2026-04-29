#include "projection/semantic_point_labeler.h"

#include "projection/point_cloud_projector.h"

namespace segment_projection::projection {

bool LookupSemanticLabelForPoint(
    const segment_projection::data_loader::GacPcdPoint& point,
    const FrontWideCameraModel& camera_model, const cv::Mat& semantic_image,
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

}  // namespace segment_projection::projection
