#pragma once

#include <opencv2/core.hpp>

#include "projection/front_wide_projection_types.h"

namespace segment_projection::projection {

bool UndistortRenderedImage(const cv::Mat& source_image,
                            const CameraModel& camera_model,
                            cv::Mat* undistorted_image);

}  // namespace segment_projection::projection
