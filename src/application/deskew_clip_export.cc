#define PCL_NO_PRECOMPILE

#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>
#include <pcl/io/pcd_io.h>

#include "application/deskew_clip_export_config.h"
#include "application/deskew_global_frame.h"
#include "data_loader/gac_clip_root_loader.h"
#include "data_loader/gac_pcd_point.h"
#include "data_loader/semantic_labeled_point.h"
#include "pose_parse/inspvax_asc_parser.h"
#include "projection/camera_calibration.h"
#include "projection/front_wide_projection_types.h"
#include "projection/image_index.h"
#include "projection/point_cloud_classification.h"
#include "projection/point_cloud_projector.h"
#include "projection/rendered_image_undistorter.h"
#include "projection/semantic_label_mapping.h"
#include "projection/semantic_point_labeler.h"

namespace {

using ClipLoader = segment_projection::data_loader::GacClipRootLoader;
using GacPcdPoint = segment_projection::data_loader::GacPcdPoint;
using SemanticLabeledPoint =
    segment_projection::data_loader::SemanticLabeledPoint;
using InspvaxSample = segment_projection::pose_parse::InspvaxSample;
using CameraModel = segment_projection::projection::CameraModel;
using ImageIndex = segment_projection::projection::ImageIndex;
using ImageMatch = segment_projection::projection::ImageMatch;
using ProjectionRenderConfig =
    segment_projection::projection::ProjectionRenderConfig;
using SemanticLookupContext =
    segment_projection::projection::SemanticLookupContext;
using ImageProjectionModel =
    segment_projection::projection::ImageProjectionModel;

struct InterpolatedPose {
  Eigen::Vector3d utm_m = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

struct CameraProjectionContext {
  std::string camera_name;
  CameraModel camera_model;
  std::unique_ptr<ImageIndex> image_index;
  std::filesystem::path output_dir;
};

struct ProjectionContext {
  std::vector<CameraProjectionContext> camera_contexts;
  ImageProjectionModel image_model = ImageProjectionModel::kUndistorted;
  ProjectionRenderConfig render_config;
  segment_projection::projection::SemanticLabelMapping semantic_mapping;
  double max_time_diff_ms = 100.0;
};

std::string TimestampNameFromPcdPath(const std::filesystem::path& pcd_path,
                                     int64_t fallback_ms) {
  const std::string stem = pcd_path.stem().string();
  if (!stem.empty()) {
    return stem;
  }
  return std::to_string(fallback_ms);
}

bool LoadPcdCloud(const std::string& pcd_path,
                  pcl::PointCloud<GacPcdPoint>* cloud) {
  if (!cloud) {
    return false;
  }
  if (pcl::io::loadPCDFile(pcd_path, *cloud) != 0) {
    LOG(ERROR) << "Failed to load PCD: " << pcd_path;
    return false;
  }
  return true;
}

bool LoadSemanticMapping(
    const std::filesystem::path& mapping_path,
    segment_projection::projection::SemanticLabelMapping* mapping) {
  if (!mapping) {
    return false;
  }
  return mapping->LoadFromJson(mapping_path);
}

const InspvaxSample* FindNearestSample(const std::vector<InspvaxSample>& samples,
                                       int64_t target_ms,
                                       int64_t max_diff_ms) {
  if (samples.empty()) {
    return nullptr;
  }

  auto it = std::lower_bound(
      samples.begin(), samples.end(), target_ms,
      [](const InspvaxSample& sample, int64_t value) {
        return sample.timestamp_ms < value;
      });

  const InspvaxSample* best = nullptr;
  int64_t best_diff = std::numeric_limits<int64_t>::max();
  if (it != samples.end()) {
    best = &(*it);
    best_diff = std::llabs(it->timestamp_ms - target_ms);
  }
  if (it != samples.begin()) {
    const auto prev = std::prev(it);
    const int64_t diff = std::llabs(prev->timestamp_ms - target_ms);
    if (diff < best_diff) {
      best = &(*prev);
      best_diff = diff;
    }
  }
  if (!best || best_diff > max_diff_ms) {
    return nullptr;
  }
  return best;
}

bool InterpolatePose(const std::vector<InspvaxSample>& samples,
                     double target_ms,
                     double max_gap_ms,
                     InterpolatedPose* pose) {
  if (!pose || samples.empty()) {
    return false;
  }

  auto it = std::lower_bound(
      samples.begin(), samples.end(), target_ms,
      [](const InspvaxSample& sample, double value) {
        return static_cast<double>(sample.timestamp_ms) < value;
      });

  if (it == samples.begin() || it == samples.end()) {
    return false;
  }

  const auto& s1 = *std::prev(it);
  const auto& s2 = *it;
  const double t1 = static_cast<double>(s1.timestamp_ms);
  const double t2 = static_cast<double>(s2.timestamp_ms);
  const double gap = t2 - t1;
  if (gap <= 0.0 || gap > max_gap_ms) {
    return false;
  }

  const double alpha = (target_ms - t1) / gap;
  pose->utm_m = s1.utm_m + alpha * (s2.utm_m - s1.utm_m);
  pose->orientation = s1.orientation.slerp(alpha, s2.orientation).normalized();
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;

  if (argc < 3 || argc > 4) {
    LOG(ERROR) << "Usage: " << argv[0]
               << " <config_yaml> <clip_root> [output_dir]";
    return EXIT_FAILURE;
  }

  const std::filesystem::path config_path = argv[1];
  const std::filesystem::path clip_root = argv[2];
  const std::filesystem::path output_dir =
      (argc >= 4) ? std::filesystem::path(argv[3]) : (clip_root / "output");

  segment_projection::application::DeskewClipExportConfig cfg;
  std::string config_error;
  if (!segment_projection::application::LoadDeskewClipExportConfig(
          config_path, &cfg, &config_error)) {
    LOG(ERROR) << config_error;
    return EXIT_FAILURE;
  }

  ClipLoader clip(clip_root);
  clip.SetOutputDir(output_dir);

  const std::filesystem::path asc_path = clip.IePostTrajAscPath();
  const std::filesystem::path lidar_dir = clip.LidarRawTopDir();
  const std::filesystem::path output_pcd_dir = clip.output_dir() / cfg.output_subdir;

  std::error_code ec;
  std::filesystem::create_directories(output_pcd_dir, ec);
  if (ec) {
    LOG(ERROR) << "Failed to create output directory: " << output_pcd_dir
               << ", error=" << ec.message();
    return EXIT_FAILURE;
  }

  std::unique_ptr<ProjectionContext> projection;
  if (cfg.projection.enabled) {
    projection = std::make_unique<ProjectionContext>();
    projection->image_model =
        (cfg.projection.image_model ==
         segment_projection::application::ProjectionImageModel::kRaw)
            ? ImageProjectionModel::kRaw
            : ImageProjectionModel::kUndistorted;
    projection->camera_contexts.reserve(cfg.projection.camera_names.size());
    for (const std::string& camera_name : cfg.projection.camera_names) {
      CameraProjectionContext camera_context;
      camera_context.camera_name = camera_name;
      if (!segment_projection::projection::LoadCameraModel(
              camera_name, clip.LidarTopToCarPath(),
              clip.CameraToCarPath(camera_name),
              &camera_context.camera_model)) {
        LOG(ERROR) << "Failed to load camera model. camera_name="
                   << camera_name;
        return EXIT_FAILURE;
      }

      camera_context.image_index = ImageIndex::Build(
          clip.CameraImageDir(cfg.projection.image_root_subdir, camera_name));
      if (!camera_context.image_index) {
        LOG(ERROR) << "Failed to build image index. camera_name="
                   << camera_name;
        return EXIT_FAILURE;
      }

      camera_context.output_dir =
          clip.output_dir() / cfg.projection.output_subdir / camera_name;
      std::filesystem::create_directories(camera_context.output_dir, ec);
      if (ec) {
        LOG(ERROR) << "Failed to create projection output directory: "
                   << camera_context.output_dir
                   << ", camera_name=" << camera_name
                   << ", error=" << ec.message();
        return EXIT_FAILURE;
      }
      projection->camera_contexts.push_back(std::move(camera_context));
    }

    projection->render_config.point_radius_px = cfg.projection.point_radius_px;
    projection->render_config.intensity_color_map =
        cfg.projection.intensity_color_map;
    projection->max_time_diff_ms = cfg.projection.max_time_diff_ms;
    const std::filesystem::path semantic_mapping_path =
        config_path.parent_path().parent_path() /
        "class_to_grayscale_mapping_panoptic.json";
    if (!LoadSemanticMapping(semantic_mapping_path,
                             &projection->semantic_mapping)) {
      LOG(ERROR) << "Failed to load semantic mapping json: "
                 << semantic_mapping_path;
      return EXIT_FAILURE;
    }

    LOG(INFO) << "Projection enabled. camera_count="
              << projection->camera_contexts.size();
    for (const auto& camera_context : projection->camera_contexts) {
      LOG(INFO) << "Projection camera ready. camera_name="
                << camera_context.camera_name
                << ", image_dir=" << camera_context.image_index->image_dir()
                << ", image_count="
                << camera_context.image_index->entries().size()
                << ", output_dir=" << camera_context.output_dir;
    }
  }

  segment_projection::pose_parse::InspvaxAscParser parser;
  segment_projection::pose_parse::InspvaxAscParser::Options parser_options;
  parser_options.apply_utm_convergence =
      segment_projection::application::ShouldApplyUtmConvergence(
          cfg.global_frame);
  parser.SetOptions(parser_options);
  segment_projection::pose_parse::InspvaxAscParser::Summary summary;
  if (!parser.ParseFile(asc_path.string(), summary)) {
    LOG(ERROR) << "Failed to parse ASC: " << asc_path;
    return EXIT_FAILURE;
  }
  std::vector<InspvaxSample> samples = parser.Samples();
  LOG(INFO) << "Parsed INSPVAX samples: " << samples.size()
            << ", skipped=" << summary.skipped
            << ", pose.coord_frame="
            << segment_projection::application::GlobalFrameName(
                   cfg.global_frame)
            << ", apply_utm_convergence="
            << (parser.options().apply_utm_convergence ? "true" : "false");

  Eigen::Matrix4d T_gnss_lidar = Eigen::Matrix4d::Identity();
  const std::filesystem::path calib_path =
      segment_projection::application::SelectGnssToLidarCalibrationPath(
          clip, cfg.global_frame);
  if (!ClipLoader::LoadGnssToLidarTop(calib_path, &T_gnss_lidar)) {
    LOG(ERROR) << "Failed to load GNSS->LiDAR extrinsic: " << calib_path;
    return EXIT_FAILURE;
  }

  const auto pcd_files = ClipLoader::GetPcdFilesWithTimestampsMs(lidar_dir);
  if (pcd_files.empty()) {
    LOG(ERROR) << "No PCD files found in " << lidar_dir;
    return EXIT_FAILURE;
  }
  std::string rewrite_error;
  if (!segment_projection::application::RewriteSamplesForGlobalFrame(
          parser, cfg.global_frame, pcd_files.begin()->first - 200, &samples,
          &rewrite_error)) {
    LOG(ERROR) << "Failed to rewrite samples for global frame "
               << segment_projection::application::GlobalFrameName(
                      cfg.global_frame)
               << ": " << rewrite_error;
    return EXIT_FAILURE;
  }

  const double deskew_max_r2 =
      (cfg.deskew_max_range_m > 0.0)
          ? cfg.deskew_max_range_m * cfg.deskew_max_range_m
          : std::numeric_limits<double>::infinity();

  int64_t frame_index = 0;
  int64_t exported_frames = 0;
  int64_t skipped_frames = 0;

  for (const auto& [timestamp_ms, pcd_path] : pcd_files) {
    if (cfg.frame_stride > 1 && (frame_index % cfg.frame_stride) != 0) {
      ++frame_index;
      continue;
    }
    ++frame_index;

    const auto* frame_sample = FindNearestSample(samples, timestamp_ms, 200);
    if (!frame_sample) {
      LOG(WARNING) << "Skip frame without nearby pose: " << pcd_path;
      ++skipped_frames;
      continue;
    }

    pcl::PointCloud<GacPcdPoint> cloud;
    if (!LoadPcdCloud(pcd_path.string(), &cloud)) {
      ++skipped_frames;
      continue;
    }

    Eigen::Matrix4d T_w_gnss_frame = Eigen::Matrix4d::Identity();
    T_w_gnss_frame.block<3, 3>(0, 0) =
        frame_sample->orientation.toRotationMatrix();
    T_w_gnss_frame.block<3, 1>(0, 3) = frame_sample->utm_m;
    const Eigen::Matrix4d T_w_lidar_frame = T_w_gnss_frame * T_gnss_lidar;
    const Eigen::Matrix4d T_lidar_frame_w = T_w_lidar_frame.inverse();
    const Eigen::Matrix3d R_lidar_frame_w =
        T_lidar_frame_w.block<3, 3>(0, 0);
    const Eigen::Vector3d t_lidar_frame_w =
        T_lidar_frame_w.block<3, 1>(0, 3);

    pcl::PointCloud<GacPcdPoint> deskewed_cloud;
    deskewed_cloud.reserve(cloud.size());

    for (const auto& point : cloud) {
      const double r2 =
          point.x * point.x + point.y * point.y + point.z * point.z;
      if (r2 > deskew_max_r2) {
        continue;
      }

      const double point_ts_ms =
          static_cast<double>(timestamp_ms) +
          static_cast<double>(point.point_time_offset) * 1e-3;

      InterpolatedPose pose;
      if (!InterpolatePose(samples, point_ts_ms, cfg.interp_max_gap_ms, &pose)) {
        continue;
      }

      Eigen::Matrix4d T_w_gnss = Eigen::Matrix4d::Identity();
      T_w_gnss.block<3, 3>(0, 0) = pose.orientation.toRotationMatrix();
      T_w_gnss.block<3, 1>(0, 3) = pose.utm_m;
      const Eigen::Matrix4d T_w_lidar = T_w_gnss * T_gnss_lidar;

      const Eigen::Vector3d p_lidar(point.x, point.y, point.z);
      const Eigen::Vector3d p_world =
          T_w_lidar.block<3, 3>(0, 0) * p_lidar + T_w_lidar.block<3, 1>(0, 3);
      const Eigen::Vector3d p_lidar_frame =
          R_lidar_frame_w * p_world + t_lidar_frame_w;

      GacPcdPoint out_point = point;
      out_point.x = static_cast<float>(p_lidar_frame.x());
      out_point.y = static_cast<float>(p_lidar_frame.y());
      out_point.z = static_cast<float>(p_lidar_frame.z());
      deskewed_cloud.push_back(out_point);
    }

    deskewed_cloud.width = static_cast<std::uint32_t>(deskewed_cloud.size());
    deskewed_cloud.height = 1;
    deskewed_cloud.is_dense = false;

    std::vector<ImageMatch> image_matches;
    std::vector<cv::Mat> semantic_images;
    std::vector<SemanticLookupContext> lookup_contexts;
    if (projection) {
      image_matches.reserve(projection->camera_contexts.size());
      semantic_images.reserve(projection->camera_contexts.size());
      lookup_contexts.reserve(projection->camera_contexts.size());
      for (const auto& camera_context : projection->camera_contexts) {
        const auto image_match =
            camera_context.image_index->FindNearestOrNull(timestamp_ms);
        if (!image_match) {
          LOG(ERROR) << "No matching semantic image for frame: " << pcd_path
                     << ", camera_name=" << camera_context.camera_name;
          return EXIT_FAILURE;
        }
        if (static_cast<double>(image_match->delta_ms) >
            projection->max_time_diff_ms) {
          LOG(ERROR) << "Nearest semantic image exceeds max delta for frame: "
                     << pcd_path << ", camera_name="
                     << camera_context.camera_name
                     << ", pcd_ts_ms=" << timestamp_ms
                     << ", image=" << image_match->path
                     << ", image_ts_ms=" << image_match->timestamp_ms
                     << ", delta_ms=" << image_match->delta_ms
                     << ", max_time_diff_ms=" << projection->max_time_diff_ms;
          return EXIT_FAILURE;
        }

        cv::Mat semantic_image =
            cv::imread(image_match->path.string(), cv::IMREAD_UNCHANGED);
        if (semantic_image.empty()) {
          LOG(ERROR) << "Failed to load projection image: " << image_match->path
                     << ", camera_name=" << camera_context.camera_name;
          return EXIT_FAILURE;
        }
        if (semantic_image.type() != CV_8UC1) {
          LOG(ERROR) << "Semantic image must be single-channel grayscale: "
                     << image_match->path << ", camera_name="
                     << camera_context.camera_name
                     << ", actual_type=" << semantic_image.type();
          return EXIT_FAILURE;
        }

        image_matches.push_back(*image_match);
        semantic_images.push_back(std::move(semantic_image));
      }
      for (std::size_t i = 0; i < projection->camera_contexts.size(); ++i) {
        SemanticLookupContext lookup_context;
        lookup_context.camera_model = &projection->camera_contexts[i].camera_model;
        lookup_context.image_model = projection->image_model;
        lookup_context.semantic_image = &semantic_images[i];
        lookup_context.mapping = &projection->semantic_mapping;
        lookup_contexts.push_back(lookup_context);
      }
    }

    pcl::PointCloud<SemanticLabeledPoint> labeled_cloud;
    labeled_cloud.reserve(deskewed_cloud.size());
    for (const auto& point : deskewed_cloud) {
      int contiguous_id = -1;
      SemanticLabeledPoint labeled_point;
      labeled_point.x = point.x;
      labeled_point.y = point.y;
      labeled_point.z = point.z;
      labeled_point.intensity = point.intensity;
      labeled_point.ring = point.ring;
      labeled_point.point_time_offset = point.point_time_offset;
      labeled_point.classification = -1;

      if (projection &&
          !segment_projection::projection::LookupSemanticLabelForPointMultiCamera(
              point, lookup_contexts, &contiguous_id)) {
        LOG(ERROR) << "Failed to resolve semantic label for frame: "
                   << pcd_path;
        return EXIT_FAILURE;
      }
      labeled_point.classification =
          segment_projection::projection::MapContiguousIdToClassification(
              contiguous_id);

      labeled_cloud.push_back(labeled_point);
    }
    labeled_cloud.width = static_cast<std::uint32_t>(labeled_cloud.size());
    labeled_cloud.height = 1;
    labeled_cloud.is_dense = false;

    const std::filesystem::path output_path =
        output_pcd_dir / (TimestampNameFromPcdPath(pcd_path, timestamp_ms) + ".pcd");
    if (pcl::io::savePCDFileBinary(output_path.string(), labeled_cloud) != 0) {
      LOG(ERROR) << "Failed to save semantic-labeled PCD: " << output_path;
      ++skipped_frames;
      continue;
    }

    if (projection) {
      const std::string output_name =
          TimestampNameFromPcdPath(pcd_path, timestamp_ms) + ".png";
      for (std::size_t i = 0; i < projection->camera_contexts.size(); ++i) {
        const auto& camera_context = projection->camera_contexts[i];
        cv::Mat projected_image;
        int valid_projected_count = 0;
        if (!segment_projection::projection::RenderProjection(
                deskewed_cloud, camera_context.camera_model,
                projection->image_model, projection->render_config,
                semantic_images[i], &projected_image,
                &valid_projected_count)) {
          LOG(ERROR) << "Failed to render projection for frame: " << pcd_path
                     << ", camera_name=" << camera_context.camera_name;
          return EXIT_FAILURE;
        }

        cv::Mat output_image;
        if (projection->image_model == ImageProjectionModel::kRaw) {
          if (!segment_projection::projection::UndistortRenderedImage(
                  projected_image, camera_context.camera_model,
                  &output_image)) {
            LOG(ERROR) << "Failed to undistort rendered projection for frame: "
                       << pcd_path << ", camera_name="
                       << camera_context.camera_name;
            return EXIT_FAILURE;
          }
        } else {
          output_image = projected_image;
        }

        const std::filesystem::path projection_output_path =
            camera_context.output_dir / output_name;
        if (!cv::imwrite(projection_output_path.string(), output_image)) {
          LOG(ERROR) << "Failed to save projection PNG: "
                     << projection_output_path
                     << ", camera_name=" << camera_context.camera_name;
          return EXIT_FAILURE;
        }

        LOG(INFO) << "Projected " << pcd_path.filename() << " -> "
                  << projection_output_path << ", camera_name="
                  << camera_context.camera_name
                  << ", projected_points=" << valid_projected_count
                  << ", image_delta_ms=" << image_matches[i].delta_ms;
      }
    }

    LOG(INFO) << "Deskewed " << pcd_path.filename() << " -> " << output_path
              << ", points=" << deskewed_cloud.size();
    ++exported_frames;
  }

  LOG(INFO) << "Deskew complete. exported_frames=" << exported_frames
            << ", skipped_frames=" << skipped_frames
            << ", output_dir=" << output_pcd_dir;
  return EXIT_SUCCESS;
}
