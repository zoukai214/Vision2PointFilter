# Front-Wide Calibration No-Undistort Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the front-wide calibration loader read and validate only `camera-front-wide` intrinsics and `camera-front-wide-to-car` extrinsics, while ignoring undistort and distortion-related fields.

**Architecture:** Keep `LoadFrontWideCameraModel` and `FrontWideCameraModel` unchanged at the interface level. Update only the JSON key paths in the loader and align the focused regression test so it proves the loader succeeds with the new keys, ignores unrelated distortion data, and still rejects invalid image size.

**Tech Stack:** C++17, jsoncpp, Eigen, existing local test executable under `tests/projection`

---

### Task 1: Rewrite the regression test to the new calibration schema

**Files:**
- Modify: `tests/projection/camera_calibration_test.cc`
- Test: `tests/projection/camera_calibration_test.cc`

- [ ] **Step 1: Write the failing test**

```cpp
if (!WriteTextFile(
        camera_path,
        R"json({
  "camera-front-wide": {
    "param": {
      "cam_matrix": {
        "data": [
          [1344.4, 0.0, 1910.5],
          [0.0, 1344.4, 1080.25],
          [0.0, 0.0, 1.0]
        ]
      },
      "width": 3840,
      "height": 2160,
      "distortion": {
        "data": [0.1, -0.2, 0.0, 0.0, 0.0]
      }
    }
  },
  "camera-front-wide-to-car": {
    "param": {
      "sensor_calib": {
        "data": [
          [0.0, -1.0, 0.0, 2.0],
          [1.0, 0.0, 0.0, 0.5],
          [0.0, 0.0, 1.0, 1.25],
          [0.0, 0.0, 0.0, 1.0]
        ]
      }
    }
  },
  "camera-front-wide-undistort": {
    "param": {
      "cam_matrix": {
        "data": [
          [1.0, 0.0, 1.0],
          [0.0, 1.0, 1.0],
          [0.0, 0.0, 1.0]
        ]
      }
    }
  }
})json")) {
  std::cerr << "failed to write fixture files\n";
  return 1;
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build --target camera_calibration_test && ./build/tests/projection/camera_calibration_test`

Expected: FAIL because the loader still looks up `camera-front-wide-undistort` and `camera-front-wide-to-car-undistort`, so it does not accept a fixture that only provides the new required keys.

- [ ] **Step 3: Extend the invalid-size fixture to the new schema**

```cpp
if (!WriteTextFile(
        camera_path,
        R"json({
  "camera-front-wide": {
    "param": {
      "cam_matrix": {
        "data": [
          [1344.4, 0.0, 1910.5],
          [0.0, 1344.4, 1080.25],
          [0.0, 0.0, 1.0]
        ]
      },
      "width": 0,
      "height": -1
    }
  },
  "camera-front-wide-to-car": {
    "param": {
      "sensor_calib": {
        "data": [
          [0.0, -1.0, 0.0, 2.0],
          [1.0, 0.0, 0.0, 0.5],
          [0.0, 0.0, 1.0, 1.25],
          [0.0, 0.0, 0.0, 1.0]
        ]
      }
    }
  }
})json")) {
  std::cerr << "failed to write invalid camera fixture\n";
  return 1;
}
```

- [ ] **Step 4: Re-run the test to keep the failure focused on loader behavior**

Run: `cmake --build build --target camera_calibration_test && ./build/tests/projection/camera_calibration_test`

Expected: Still FAIL before production code changes, for the same old-key lookup reason.

### Task 2: Switch the loader to only the required keys

**Files:**
- Modify: `src/projection/camera_calibration.h`
- Test: `tests/projection/camera_calibration_test.cc`

- [ ] **Step 1: Update the extrinsics JSON path**

```cpp
if (!segment_projection::data_loader::GacClipRootLoader::LoadCalibMatrix(
        camera_front_wide_to_car_path,
        {"camera-front-wide-to-car", "param", "sensor_calib", "data"},
        &loaded_model.T_car_cam)) {
  return false;
}
```

- [ ] **Step 2: Update the intrinsics and image-size JSON paths**

```cpp
if (!detail::LoadMatrix3d(root,
                          {"camera-front-wide", "param",
                           "cam_matrix", "data"},
                          &loaded_model.K)) {
  return false;
}
if (!detail::LoadInt(root,
                     {"camera-front-wide", "param", "width"},
                     &loaded_model.image_width)) {
  return false;
}
if (!detail::LoadInt(root,
                     {"camera-front-wide", "param", "height"},
                     &loaded_model.image_height)) {
  return false;
}
```

- [ ] **Step 3: Do not add any distortion parsing**

```cpp
// No distortion lookup is added here. The loader ignores unrelated fields
// and only validates the required intrinsics, image size, and extrinsics.
```

- [ ] **Step 4: Run the focused test to verify it passes**

Run: `cmake --build build --target camera_calibration_test && ./build/tests/projection/camera_calibration_test`

Expected: PASS with exit code 0.

### Task 3: Verify no runtime undistort dependency remains

**Files:**
- Modify: `src/projection/camera_calibration.h`
- Modify: `tests/projection/camera_calibration_test.cc`

- [ ] **Step 1: Search for remaining front-wide undistort runtime references**

Run: `grep -RIn --exclude-dir=.git -E "camera-front-wide-undistort|camera-front-wide-to-car-undistort" src tests`

Expected: Only documentation references may remain. Runtime loader and its test fixture should no longer depend on undistort keys.

- [ ] **Step 2: Re-run the focused regression test as completion evidence**

Run: `cmake --build build --target camera_calibration_test && ./build/tests/projection/camera_calibration_test`

Expected: PASS with exit code 0.
