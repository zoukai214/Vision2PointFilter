# Configurable Camera Semantic Projection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the single-camera projection path in `deskew_clip_export` with an explicit `camera_names`-driven multi-camera pipeline that renders one PNG per configured camera and writes one fused `semantic_label` per exported point.

**Architecture:** First extract projection configuration and camera path derivation into testable units, because the current config parser is buried inside `deskew_clip_export.cc`. Then generalize the projection stack around a reusable `CameraModel` and an ordered multi-camera semantic lookup API. Finally wire `deskew_clip_export` to build one projection context per configured camera, load each image independently, fuse semantic labels in config order, and write per-camera PNG outputs.

**Tech Stack:** C++17, Eigen, OpenCV, PCL, yaml-cpp, jsoncpp, glog, CMake/CTest

---

### Task 1: Extract and validate the new multi-camera projection config

**Files:**
- Create: `src/application/deskew_clip_export_config.h`
- Create: `src/application/deskew_clip_export_config.cc`
- Modify: `src/application/deskew_clip_export.cc`
- Modify: `src/data_loader/gac_clip_root_loader.h`
- Modify: `src/data_loader/gac_clip_root_loader.cc`
- Modify: `config/deskew_clip_export.yaml`
- Modify: `config/deskew_clip_export_utm.yaml`
- Test: `tests/application/deskew_clip_export_config_test.cc`

- [ ] **Step 1: Write the failing config parsing and camera path tests**

```cpp
// tests/application/deskew_clip_export_config_test.cc
#include "application/deskew_clip_export_config.h"
#include "data_loader/gac_clip_root_loader.h"

int main() {
  const std::filesystem::path temp_dir =
      std::filesystem::temp_directory_path() /
      "deskew_clip_export_config_test";
  std::error_code ec;
  std::filesystem::remove_all(temp_dir, ec);
  std::filesystem::create_directories(temp_dir);

  const std::filesystem::path config_path = temp_dir / "config.yaml";
  {
    std::ofstream ofs(config_path);
    ofs << "deskew_clip_export:\n";
    ofs << "  projection:\n";
    ofs << "    enabled: true\n";
    ofs << "    image_root_subdir: images_seg_mask2former\n";
    ofs << "    camera_names: [front_wide, back, left_front]\n";
    ofs << "    output_subdir: projection\n";
    ofs << "    max_time_diff_ms: 120.0\n";
    ofs << "    point_radius_px: 3\n";
    ofs << "    intensity_color_map: turbo\n";
  }

  segment_projection::application::DeskewClipExportConfig cfg;
  std::string error;
  if (!segment_projection::application::LoadDeskewClipExportConfig(
          config_path, &cfg, &error)) {
    std::cerr << "LoadDeskewClipExportConfig failed: " << error << "\n";
    return 1;
  }
  if (cfg.projection.camera_names.size() != 3 ||
      cfg.projection.camera_names[1] != "back") {
    std::cerr << "unexpected camera_names order\n";
    return 1;
  }
  if (cfg.projection.image_root_subdir != "images_seg_mask2former") {
    std::cerr << "unexpected image_root_subdir\n";
    return 1;
  }

  const std::filesystem::path duplicate_path = temp_dir / "duplicate.yaml";
  {
    std::ofstream ofs(duplicate_path);
    ofs << "deskew_clip_export:\n";
    ofs << "  projection:\n";
    ofs << "    enabled: true\n";
    ofs << "    image_root_subdir: images_seg_mask2former\n";
    ofs << "    camera_names: [front_wide, front_wide]\n";
    ofs << "    output_subdir: projection\n";
    ofs << "    max_time_diff_ms: 100.0\n";
    ofs << "    point_radius_px: 2\n";
    ofs << "    intensity_color_map: turbo\n";
  }
  if (segment_projection::application::LoadDeskewClipExportConfig(
          duplicate_path, &cfg, &error)) {
    std::cerr << "duplicate camera_names should fail\n";
    return 1;
  }

  segment_projection::data_loader::GacClipRootLoader clip(temp_dir / "clip");
  const auto image_dir =
      clip.CameraImageDir("images_seg_mask2former", "left_front");
  const auto calib_path = clip.CameraToCarPath("left_front");
  if (image_dir.filename() != "left_front") {
    std::cerr << "unexpected image_dir: " << image_dir << "\n";
    return 1;
  }
  if (calib_path.filename() != "calib_camera_left_front_to_car.json") {
    std::cerr << "unexpected calib_path: " << calib_path << "\n";
    return 1;
  }
  return 0;
}
```

- [ ] **Step 2: Run the new config test and verify it fails**

Run: `cmake --build build --target deskew_clip_export_config_test`

Run: `ctest --test-dir build -R deskew_clip_export_config_test --output-on-failure`

Expected: build failure because `DeskewClipExportConfig`, `LoadDeskewClipExportConfig`, `CameraImageDir`, or `CameraToCarPath` do not exist yet.

- [ ] **Step 3: Implement the extracted config module and generic camera path helpers**

```cpp
// src/application/deskew_clip_export_config.h
namespace segment_projection::application {

struct ProjectionConfig {
  bool enabled = true;
  std::string image_root_subdir = "images_seg_mask2former";
  std::vector<std::string> camera_names;
  std::string output_subdir = "projection";
  double max_time_diff_ms = 100.0;
  int point_radius_px = 2;
  std::string intensity_color_map = "turbo";
};

struct DeskewClipExportConfig {
  GlobalFrame global_frame = GlobalFrame::kEnu;
  int frame_stride = 1;
  double deskew_max_range_m = 100.0;
  double interp_max_gap_ms = 100.0;
  std::string output_subdir = "deskew_pcd";
  ProjectionConfig projection;
};

bool LoadDeskewClipExportConfig(const std::filesystem::path& config_path,
                                DeskewClipExportConfig* cfg,
                                std::string* error);

}  // namespace segment_projection::application
```

```cpp
// src/data_loader/gac_clip_root_loader.h
std::filesystem::path CameraToCarPath(const std::string& camera_name) const;
std::filesystem::path CameraImageDir(const std::string& image_root_subdir,
                                     const std::string& camera_name) const;
```

```cpp
// src/data_loader/gac_clip_root_loader.cc
std::filesystem::path GacClipRootLoader::CameraToCarPath(
    const std::string& camera_name) const {
  return CalibExtractDir() /
         ("calib_camera_" + camera_name + "_to_car.json");
}

std::filesystem::path GacClipRootLoader::CameraImageDir(
    const std::string& image_root_subdir,
    const std::string& camera_name) const {
  const std::filesystem::path root_path(image_root_subdir);
  const std::filesystem::path full_path =
      root_path.is_relative() ? (clip_root_ / root_path / camera_name)
                              : (root_path / camera_name);
  return std::filesystem::absolute(full_path);
}
```

- [ ] **Step 4: Update `deskew_clip_export.cc` and YAML samples to use the new config type**

```cpp
// src/application/deskew_clip_export.cc
#include "application/deskew_clip_export_config.h"

segment_projection::application::DeskewClipExportConfig cfg;
std::string config_error;
if (!segment_projection::application::LoadDeskewClipExportConfig(
        config_path, &cfg, &config_error)) {
  LOG(ERROR) << config_error;
  return EXIT_FAILURE;
}
```

```yaml
# config/deskew_clip_export.yaml
deskew_clip_export:
  projection:
    enabled: true
    image_root_subdir: images_seg_mask2former
    camera_names: [front_wide, back, front_narrow, left_back, left_front, right_back, right_front]
    output_subdir: projection
    max_time_diff_ms: 100.0
    point_radius_px: 2
    intensity_color_map: turbo
```

- [ ] **Step 5: Run the config test again and verify it passes**

Run: `cmake --build build --target deskew_clip_export_config_test`

Run: `ctest --test-dir build -R deskew_clip_export_config_test --output-on-failure`

Expected: PASS, including duplicate camera name rejection and generic path derivation.

- [ ] **Step 6: Commit the config extraction**

```bash
git add src/application/deskew_clip_export_config.h \
        src/application/deskew_clip_export_config.cc \
        src/application/deskew_clip_export.cc \
        src/data_loader/gac_clip_root_loader.h \
        src/data_loader/gac_clip_root_loader.cc \
        config/deskew_clip_export.yaml \
        config/deskew_clip_export_utm.yaml \
        tests/application/deskew_clip_export_config_test.cc
git commit -m "refactor: extract configurable projection config"
```

### Task 2: Generalize the camera model and projection helpers

**Files:**
- Modify: `src/projection/front_wide_projection_types.h`
- Modify: `src/projection/camera_calibration.h`
- Modify: `src/projection/point_cloud_projector.h`
- Modify: `src/projection/point_cloud_projector.cc`
- Modify: `tests/projection/camera_calibration_test.cc`
- Modify: `tests/projection/point_cloud_projector_test.cc`

- [ ] **Step 1: Extend the existing projection tests to target generic camera APIs**

```cpp
// tests/projection/camera_calibration_test.cc
segment_projection::projection::CameraModel front_model;
if (!segment_projection::projection::LoadCameraModel(
        "front_wide", lidar_path, front_camera_path, &front_model)) {
  std::cerr << "LoadCameraModel(front_wide) failed\n";
  return 1;
}
if (front_model.camera_name != "front_wide") {
  std::cerr << "unexpected camera_name\n";
  return 1;
}

segment_projection::projection::CameraModel left_front_model;
if (!segment_projection::projection::LoadCameraModel(
        "left_front", lidar_path, left_front_camera_path,
        &left_front_model)) {
  std::cerr << "LoadCameraModel(left_front) failed\n";
  return 1;
}
if (left_front_model.image_width != 1920) {
  std::cerr << "unexpected left_front width\n";
  return 1;
}
```

```cpp
// tests/projection/point_cloud_projector_test.cc
using segment_projection::projection::CameraModel;

CameraModel camera_model;
camera_model.camera_name = "back";
camera_model.T_car_lidar = Eigen::Matrix4d::Identity();
camera_model.T_lidar_car = Eigen::Matrix4d::Identity();
camera_model.T_car_cam = Eigen::Matrix4d::Identity();
camera_model.T_cam_car = Eigen::Matrix4d::Identity();
camera_model.K << 10.0, 0.0, 5.0,
                  0.0, 10.0, 5.0,
                  0.0, 0.0, 1.0;
camera_model.image_width = 10;
camera_model.image_height = 10;

const bool ok = segment_projection::projection::RenderProjection(
    cloud, camera_model, config, input_image, &output_image, &valid_count);
if (!ok) {
  std::cerr << "RenderProjection returned false\n";
  return 1;
}
```

- [ ] **Step 2: Run the calibration and projector tests and verify they fail**

Run: `cmake --build build --target camera_calibration_test point_cloud_projector_test`

Run: `ctest --test-dir build -R 'camera_calibration_test|point_cloud_projector_test' --output-on-failure`

Expected: build or test failure because `CameraModel`, `LoadCameraModel`, or `RenderProjection` do not exist yet.

- [ ] **Step 3: Implement `CameraModel`, `LoadCameraModel`, and `RenderProjection`**

```cpp
// src/projection/front_wide_projection_types.h
struct CameraModel {
  std::string camera_name;
  Eigen::Matrix4d T_lidar_car = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_car_lidar = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_car_cam = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_cam_car = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  int image_width = 0;
  int image_height = 0;
};
```

```cpp
// src/projection/camera_calibration.h
inline std::string ToCameraKey(const std::string& camera_name) {
  std::string key = camera_name;
  std::replace(key.begin(), key.end(), '_', '-');
  return key;
}

inline bool LoadCameraModel(const std::string& camera_name,
                            const std::filesystem::path& lidar_top_to_car_path,
                            const std::filesystem::path& camera_to_car_path,
                            CameraModel* model) {
  if (!model || camera_name.empty()) {
    return false;
  }
  CameraModel loaded_model;
  loaded_model.camera_name = camera_name;
  const std::string camera_key = ToCameraKey(camera_name);
  const std::string camera_node = "camera-" + camera_key;
  const std::string extrinsic_node = camera_node + "-to-car";
  if (!segment_projection::data_loader::GacClipRootLoader::LoadCalibMatrix(
          lidar_top_to_car_path,
          {"lidar-top-to-car", "param", "sensor_calib", "data"},
          &loaded_model.T_car_lidar)) {
    return false;
  }
  loaded_model.T_lidar_car =
      segment_projection::data_loader::GacClipRootLoader::InvertRigidTransform(
          loaded_model.T_car_lidar);
  if (!segment_projection::data_loader::GacClipRootLoader::LoadCalibMatrix(
          camera_to_car_path,
          {extrinsic_node, "param", "sensor_calib", "data"},
          &loaded_model.T_car_cam)) {
    return false;
  }
  loaded_model.T_cam_car =
      segment_projection::data_loader::GacClipRootLoader::InvertRigidTransform(
          loaded_model.T_car_cam);
  Json::Value root;
  if (!detail::LoadJson(camera_to_car_path, &root) ||
      !detail::LoadMatrix3d(root, {camera_node, "param", "cam_matrix", "data"},
                            &loaded_model.K) ||
      !detail::LoadInt(root, {camera_node, "param", "width"},
                       &loaded_model.image_width) ||
      !detail::LoadInt(root, {camera_node, "param", "height"},
                       &loaded_model.image_height) ||
      loaded_model.image_width <= 0 || loaded_model.image_height <= 0) {
    return false;
  }
  *model = loaded_model;
  return true;
}
```

```cpp
// src/projection/point_cloud_projector.h
bool ProjectLidarPointToPixel(
    const segment_projection::data_loader::GacPcdPoint& point,
    const CameraModel& camera_model, cv::Point* pixel);

bool RenderProjection(
    const pcl::PointCloud<segment_projection::data_loader::GacPcdPoint>& cloud,
    const CameraModel& camera_model,
    const ProjectionRenderConfig& config, const cv::Mat& input_image,
    cv::Mat* output_image, int* valid_projected_count);
```

- [ ] **Step 4: Run the projection tests and verify they pass**

Run: `cmake --build build --target camera_calibration_test point_cloud_projector_test`

Run: `ctest --test-dir build -R 'camera_calibration_test|point_cloud_projector_test' --output-on-failure`

Expected: PASS, including camera-name-based calibration loading and generic rendering.

- [ ] **Step 5: Commit the generalized projection layer**

```bash
git add src/projection/front_wide_projection_types.h \
        src/projection/camera_calibration.h \
        src/projection/point_cloud_projector.h \
        src/projection/point_cloud_projector.cc \
        tests/projection/camera_calibration_test.cc \
        tests/projection/point_cloud_projector_test.cc
git commit -m "refactor: generalize camera projection helpers"
```

### Task 3: Add ordered multi-camera semantic fusion

**Files:**
- Modify: `src/projection/semantic_point_labeler.h`
- Modify: `src/projection/semantic_point_labeler.cc`
- Modify: `tests/projection/semantic_point_labeler_test.cc`

- [ ] **Step 1: Write failing tests for ordered multi-camera semantic lookup**

```cpp
// tests/projection/semantic_point_labeler_test.cc
using segment_projection::projection::CameraModel;
using segment_projection::projection::SemanticImageView;

CameraModel front_model;
front_model.camera_name = "front_wide";
front_model.T_car_lidar = Eigen::Matrix4d::Identity();
front_model.T_lidar_car = Eigen::Matrix4d::Identity();
front_model.T_car_cam = Eigen::Matrix4d::Identity();
front_model.T_cam_car = Eigen::Matrix4d::Identity();
front_model.K << 10.0, 0.0, 5.0,
                 0.0, 10.0, 5.0,
                 0.0, 0.0, 1.0;
front_model.image_width = 10;
front_model.image_height = 10;

CameraModel back_model = front_model;
back_model.camera_name = "back";

cv::Mat front_image = cv::Mat::zeros(10, 10, CV_8UC1);
cv::Mat back_image = cv::Mat::zeros(10, 10, CV_8UC1);
front_image.at<std::uint8_t>(5, 5) = 0;
back_image.at<std::uint8_t>(5, 5) = 51;

std::vector<SemanticImageView> views = {
    {&front_model, &front_image},
    {&back_model, &back_image},
};

int semantic_label = -99;
if (!segment_projection::projection::LookupSemanticLabelForPointMultiCamera(
        MakePoint(0.0f, 0.0f, 2.0f), views, mapping, &semantic_label)) {
  std::cerr << "LookupSemanticLabelForPointMultiCamera failed\n";
  return 1;
}
if (semantic_label != 10) {
  std::cerr << "expected first valid label 10, got " << semantic_label
            << "\n";
  return 1;
}

front_image.at<std::uint8_t>(5, 5) = 255;
if (!segment_projection::projection::LookupSemanticLabelForPointMultiCamera(
        MakePoint(0.0f, 0.0f, 2.0f), views, mapping, &semantic_label)) {
  std::cerr << "LookupSemanticLabelForPointMultiCamera fallback failed\n";
  return 1;
}
if (semantic_label != 13) {
  std::cerr << "expected fallback label 13, got " << semantic_label << "\n";
  return 1;
}
```

- [ ] **Step 2: Run the semantic labeler test and verify it fails**

Run: `cmake --build build --target semantic_point_labeler_test`

Run: `ctest --test-dir build -R semantic_point_labeler_test --output-on-failure`

Expected: build failure because `SemanticImageView` and `LookupSemanticLabelForPointMultiCamera` do not exist yet.

- [ ] **Step 3: Implement the ordered multi-camera API**

```cpp
// src/projection/semantic_point_labeler.h
struct SemanticImageView {
  const CameraModel* camera_model = nullptr;
  const cv::Mat* semantic_image = nullptr;
};

bool LookupSemanticLabelForPointMultiCamera(
    const segment_projection::data_loader::GacPcdPoint& point,
    const std::vector<SemanticImageView>& views,
    const SemanticLabelMapping& mapping, int* semantic_label);
```

```cpp
// src/projection/semantic_point_labeler.cc
bool LookupSemanticLabelForPointMultiCamera(
    const segment_projection::data_loader::GacPcdPoint& point,
    const std::vector<SemanticImageView>& views,
    const SemanticLabelMapping& mapping, int* semantic_label) {
  if (!semantic_label) {
    return false;
  }
  *semantic_label = -1;
  for (const auto& view : views) {
    if (!view.camera_model || !view.semantic_image) {
      return false;
    }
    int candidate_label = -1;
    if (!LookupSemanticLabelForPoint(point, *view.camera_model,
                                     *view.semantic_image, mapping,
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
```

- [ ] **Step 4: Run the semantic labeler test and verify it passes**

Run: `cmake --build build --target semantic_point_labeler_test`

Run: `ctest --test-dir build -R semantic_point_labeler_test --output-on-failure`

Expected: PASS, including first-hit and fallback behavior in config order.

- [ ] **Step 5: Commit the semantic fusion layer**

```bash
git add src/projection/semantic_point_labeler.h \
        src/projection/semantic_point_labeler.cc \
        tests/projection/semantic_point_labeler_test.cc
git commit -m "feat: add ordered multi-camera semantic fusion"
```

### Task 4: Wire `deskew_clip_export` for per-camera rendering and fused labels

**Files:**
- Create: `src/application/deskew_clip_export_projection.h`
- Create: `src/application/deskew_clip_export_projection.cc`
- Modify: `src/application/deskew_clip_export.cc`
- Modify: `tests/application/deskew_clip_export_config_test.cc`
- Modify: `config/deskew_clip_export.yaml`
- Modify: `config/deskew_clip_export_utm.yaml`

- [ ] **Step 1: Extend the application test to cover projection context creation**

```cpp
// tests/application/deskew_clip_export_config_test.cc
std::filesystem::create_directories(temp_dir / "clip" / "images_seg_mask2former" /
                                    "front_wide");
std::filesystem::create_directories(temp_dir / "clip" / "images_seg_mask2former" /
                                    "back");
std::filesystem::create_directories(temp_dir / "clip" / "calib_extract");
std::ofstream(temp_dir / "clip" / "calib_extract" /
              "calib_camera_front_wide_to_car.json") << front_json;
std::ofstream(temp_dir / "clip" / "calib_extract" /
              "calib_camera_back_to_car.json") << back_json;

std::vector<segment_projection::application::ProjectionCameraContext> contexts;
if (!segment_projection::application::BuildProjectionCameraContexts(
        cfg, clip, &contexts, &error)) {
  std::cerr << "BuildProjectionCameraContexts failed: " << error << "\n";
  return 1;
}
if (contexts.size() != 2) {
  std::cerr << "expected 2 contexts\n";
  return 1;
}
if (contexts[0].camera_model.camera_name != "front_wide" ||
    contexts[1].output_dir.filename() != "back") {
  std::cerr << "unexpected context order or output dir\n";
  return 1;
}
```

- [ ] **Step 2: Run the application config test and verify it fails**

Run: `cmake --build build --target deskew_clip_export_config_test`

Run: `ctest --test-dir build -R deskew_clip_export_config_test --output-on-failure`

Expected: build failure because `ProjectionCameraContext` and `BuildProjectionCameraContexts` do not exist yet.

- [ ] **Step 3: Implement per-camera contexts and the new frame loop**

```cpp
// src/application/deskew_clip_export_projection.h
namespace segment_projection::application {

struct ProjectionCameraContext {
  projection::CameraModel camera_model;
  std::unique_ptr<ImageIndex> image_index;
  std::filesystem::path output_dir;
  projection::ProjectionRenderConfig render_config;
  double max_time_diff_ms = 100.0;
};

bool BuildProjectionCameraContexts(
    const DeskewClipExportConfig& cfg,
    const data_loader::GacClipRootLoader& clip,
    std::vector<ProjectionCameraContext>* contexts,
    std::string* error);

}  // namespace segment_projection::application
```

```cpp
// src/application/deskew_clip_export_projection.cc
bool BuildProjectionCameraContexts(
    const DeskewClipExportConfig& cfg,
    const data_loader::GacClipRootLoader& clip,
    std::vector<ProjectionCameraContext>* contexts,
    std::string* error) {
  if (!contexts) {
    return false;
  }
  contexts->clear();
  for (const std::string& camera_name : cfg.projection.camera_names) {
    ProjectionCameraContext context;
    if (!projection::LoadCameraModel(camera_name, clip.LidarTopToCarPath(),
                                     clip.CameraToCarPath(camera_name),
                                     &context.camera_model)) {
      *error = "Failed to load camera model: " + camera_name;
      return false;
    }
    context.image_index = projection::ImageIndex::Build(
        clip.CameraImageDir(cfg.projection.image_root_subdir, camera_name));
    if (!context.image_index) {
      *error = "Failed to build image index for camera: " + camera_name;
      return false;
    }
    context.output_dir =
        clip.output_dir() / cfg.projection.output_subdir / camera_name;
    std::filesystem::create_directories(context.output_dir);
    context.render_config.point_radius_px = cfg.projection.point_radius_px;
    context.render_config.intensity_color_map =
        cfg.projection.intensity_color_map;
    context.max_time_diff_ms = cfg.projection.max_time_diff_ms;
    contexts->push_back(std::move(context));
  }
  return true;
}
```

```cpp
// src/application/deskew_clip_export.cc
#include "application/deskew_clip_export_projection.h"

std::vector<ProjectionCameraContext> projection_contexts;
if (cfg.projection.enabled &&
    !BuildProjectionCameraContexts(cfg, clip, &projection_contexts,
                                   &config_error)) {
  LOG(ERROR) << config_error;
  return EXIT_FAILURE;
}

segment_projection::projection::SemanticLabelMapping semantic_mapping;
if (cfg.projection.enabled &&
    !LoadSemanticMapping(semantic_mapping_path, &semantic_mapping)) {
  LOG(ERROR) << "Failed to load semantic mapping json: "
             << semantic_mapping_path;
  return EXIT_FAILURE;
}

std::vector<cv::Mat> semantic_images;
std::vector<segment_projection::projection::SemanticImageView> semantic_views;
for (const auto& context : projection_contexts) {
  const auto match = context.image_index->FindNearestOrNull(timestamp_ms);
  if (!match || static_cast<double>(match->delta_ms) > context.max_time_diff_ms) {
    LOG(ERROR) << "Failed to find valid image for camera "
               << context.camera_model.camera_name;
    return EXIT_FAILURE;
  }
  semantic_images.push_back(
      cv::imread(match->path.string(), cv::IMREAD_UNCHANGED));
  semantic_views.push_back({&context.camera_model, &semantic_images.back()});
}

for (const auto& point : deskewed_cloud) {
  SemanticLabeledPoint labeled_point;
  labeled_point.semantic_label = -1;
  if (!segment_projection::projection::LookupSemanticLabelForPointMultiCamera(
          point, semantic_views, semantic_mapping,
          &labeled_point.semantic_label)) {
    LOG(ERROR) << "Failed to fuse semantic label";
    return EXIT_FAILURE;
  }
  labeled_cloud.push_back(labeled_point);
}

for (std::size_t i = 0; i < projection_contexts.size(); ++i) {
  cv::Mat projected_image;
  int valid_projected_count = 0;
  if (!segment_projection::projection::RenderProjection(
          deskewed_cloud, projection_contexts[i].camera_model,
          projection_contexts[i].render_config, semantic_images[i],
          &projected_image, &valid_projected_count)) {
    LOG(ERROR) << "Failed to render camera "
               << projection_contexts[i].camera_model.camera_name;
    return EXIT_FAILURE;
  }
  const std::filesystem::path png_path =
      projection_contexts[i].output_dir /
      (TimestampNameFromPcdPath(pcd_path, timestamp_ms) + ".png");
  if (!cv::imwrite(png_path.string(), projected_image)) {
    LOG(ERROR) << "Failed to write " << png_path;
    return EXIT_FAILURE;
  }
}
```

- [ ] **Step 4: Run focused tests and verify they pass**

Run: `cmake --build build --target deskew_clip_export_config_test semantic_point_labeler_test camera_calibration_test point_cloud_projector_test`

Run: `ctest --test-dir build -R 'deskew_clip_export_config_test|semantic_point_labeler_test|camera_calibration_test|point_cloud_projector_test' --output-on-failure`

Expected: PASS, covering config validation, camera generalization, multi-camera fusion, and per-camera output context creation.

- [ ] **Step 5: Run the full test suite for regression coverage**

Run: `ctest --test-dir build --output-on-failure`

Expected: all existing tests PASS.

- [ ] **Step 6: Commit the application wiring**

```bash
git add src/application/deskew_clip_export_projection.h \
        src/application/deskew_clip_export_projection.cc \
        src/application/deskew_clip_export.cc \
        tests/application/deskew_clip_export_config_test.cc \
        config/deskew_clip_export.yaml \
        config/deskew_clip_export_utm.yaml
git commit -m "feat: support configurable multi-camera semantic projection"
```
