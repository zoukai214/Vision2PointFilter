# Deskew Global Frame ENU UTM Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an explicit `enu|utm` global-frame option to deskew motion compensation, while keeping the shared `calib_gnss_to_lidar_top_ENU.json` extrinsic and matching `icv_mapping`-style pose semantics.

**Architecture:** Keep one deskew algorithm and branch only at configuration parsing, parser convergence behavior, and optional local-ENU sample rewriting. GNSS-to-LiDAR extrinsics continue to come from the shared ENU calibration file in both modes. Reuse `InspvaxSample::utm_m` as the world-position container whose numeric meaning becomes UTM or local ENU depending on the selected mode.

**Tech Stack:** C++17, Eigen, GDAL/OGR, yaml-cpp, jsoncpp, existing local executable tests

---

### Task 1: Add failing tests for global-frame selection and shared calibration binding

**Files:**
- Create: `tests/application/deskew_global_frame_test.cc`
- Modify: `CMakeLists.txt`
- Test: `tests/application/deskew_global_frame_test.cc`

- [ ] **Step 1: Write the failing test for config parsing and file selection**

```cpp
// Cover:
// - pose.coord_frame = "enu" is accepted
// - pose.coord_frame = "utm" is accepted
// - pose.coord_frame = "bad" is rejected
// - enu mode resolves calib_gnss_to_lidar_top_ENU.json
// - utm mode also resolves calib_gnss_to_lidar_top_ENU.json
```

- [ ] **Step 2: Register the test target**

Run: `cmake --build /workspace/Vision2PointFilter/build --target deskew_global_frame_test`

Expected: FAIL because the new test target or referenced helpers do not exist yet.

### Task 2: Add config enum and shared calibration path selection

**Files:**
- Modify: `src/application/deskew_clip_export.cc`
- Test: `tests/application/deskew_global_frame_test.cc`

- [ ] **Step 1: Add global-frame enum and config parsing**

```cpp
enum class GlobalFrame {
  kEnu,
  kUtm,
};
```

- [ ] **Step 2: Keep the shared ENU calibration path for both modes**

```cpp
const std::filesystem::path calib_path = clip.GnssToLidarTopEnuPath();
```

- [ ] **Step 3: Re-run the focused test**

Run: `cmake --build /workspace/Vision2PointFilter/build --target deskew_global_frame_test && /workspace/Vision2PointFilter/bin/deskew_global_frame_test`

Expected: Still FAIL because ENU local-pose rewriting and convergence branching are not implemented yet.

### Task 3: Add local ENU conversion and parser convergence branching

**Files:**
- Modify: `src/pose_parse/inspvax_asc_parser.h`
- Modify: `src/pose_parse/inspvax_asc_parser.cc`
- Modify: `src/application/deskew_clip_export.cc`
- Test: `tests/application/deskew_global_frame_test.cc`

- [ ] **Step 1: Add `ToLocalEnu(...)` to the parser**

```cpp
bool ToLocalEnu(double lat_deg, double lon_deg, double alt_m,
                const Eigen::Vector3d& origin_llh_deg_m,
                Eigen::Vector3d* enu_m, std::string* error) const;
```

- [ ] **Step 2: Add application helpers mirroring `icv_mapping`**

```cpp
bool ConvertSamplesToLocalEnu(...);
void RecenterSamplePositions(...);
```

- [ ] **Step 3: Branch parser options and sample rewriting by mode**

```cpp
parser_options.apply_utm_convergence = (cfg.global_frame != GlobalFrame::kEnu);
if (cfg.global_frame == GlobalFrame::kEnu) {
  ConvertSamplesToLocalEnu(...);
  RecenterSamplePositions(...);
}
```

- [ ] **Step 4: Run the focused test to verify it passes**

Run: `cmake --build /workspace/Vision2PointFilter/build --target deskew_global_frame_test && /workspace/Vision2PointFilter/bin/deskew_global_frame_test`

Expected: PASS with exit code 0.

### Task 4: Verify the deskew executable still builds with the new mode support

**Files:**
- Modify: `config/deskew_clip_export.yaml`
- Modify: `src/application/deskew_clip_export.cc`
- Test: `tests/application/deskew_global_frame_test.cc`

- [ ] **Step 1: Add the documented config field**

```yaml
pose:
  coord_frame: enu
```

- [ ] **Step 2: Rebuild the main executable**

Run: `cmake --build /workspace/Vision2PointFilter/build --target deskew_clip_export`

Expected: PASS.

- [ ] **Step 3: Re-run the focused regression test as completion evidence**

Run: `cmake --build /workspace/Vision2PointFilter/build --target deskew_global_frame_test && /workspace/Vision2PointFilter/bin/deskew_global_frame_test`

Expected: PASS with exit code 0.
