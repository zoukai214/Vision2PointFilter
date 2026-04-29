#pragma once

#include <string>

#include <opencv2/core/mat.hpp>
#include <pcl/point_cloud.h>

#include "data_loader/gac_pcd_point.h"
#include "projection/front_wide_projection_types.h"

namespace segment_projection::projection {

struct ProjectionRenderConfig {
  int point_radius_px = 2;
  std::string intensity_color_map = "turbo";
};

bool ProjectLidarPointToPixel(
    const segment_projection::data_loader::GacPcdPoint& point,
    const CameraModel& camera_model, cv::Point* pixel);

bool RenderProjection(
    const pcl::PointCloud<segment_projection::data_loader::GacPcdPoint>& cloud,
    const CameraModel& camera_model,
    const ProjectionRenderConfig& config, const cv::Mat& input_image,
    cv::Mat* output_image, int* valid_projected_count);

inline bool RenderFrontWideProjection(
    const pcl::PointCloud<segment_projection::data_loader::GacPcdPoint>& cloud,
    const FrontWideCameraModel& camera_model,
    const ProjectionRenderConfig& config, const cv::Mat& input_image,
    cv::Mat* output_image, int* valid_projected_count) {
  return RenderProjection(cloud, camera_model, config, input_image,
                          output_image, valid_projected_count);
}

}  // namespace segment_projection::projection
