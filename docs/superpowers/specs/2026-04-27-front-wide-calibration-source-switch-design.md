# Front Wide Calibration Source Switch Design

## Goal

Adjust the front-wide projection calibration loading logic so it reads only the non-`undistort` calibration entries from the camera calibration JSON file.

The required sources are:
- `camera-front-wide.param.cam_matrix.data`
- `camera-front-wide.param.width`
- `camera-front-wide.param.height`
- `camera-front-wide-to-car.param.sensor_calib.data`

## Scope

In scope:
- Update the front-wide camera model loader to read the non-`undistort` intrinsic and extrinsic entries.
- Update the calibration unit test fixtures so they validate only the non-`undistort` entries.
- Preserve the existing failure behavior when required keys are missing or malformed.

Out of scope:
- Compatibility with `camera-front-wide-undistort`
- Compatibility with `camera-front-wide-to-car-undistort`
- Any fallback logic between old and new keys
- Any changes to projection math, rendering, configuration, or executable flow

## Existing Context

The current front-wide calibration loader lives in [camera_calibration.h](/workspace/Vision2PointFilter/src/projection/camera_calibration.h). It already loads:
- `lidar-top-to-car.param.sensor_calib.data` from `calib_lidar_top_to_car.json`
- `camera-front-wide-undistort.param.cam_matrix.data`
- `camera-front-wide-undistort.param.width`
- `camera-front-wide-undistort.param.height`
- `camera-front-wide-to-car-undistort.param.sensor_calib.data`

The current unit coverage is in [camera_calibration_test.cc](/workspace/Vision2PointFilter/tests/projection/camera_calibration_test.cc), and its JSON fixtures mirror the old `undistort` key layout.

The user confirmed that real calibration files may contain both non-`undistort` and `undistort` entries in the same file, but the implementation must read only the non-`undistort` entries and ignore the `undistort` entries completely.

## Recommended Approach

Apply a narrow source-switch in the calibration loader without changing its interface.

This keeps the caller contract stable while making the actual JSON lookup paths match the required calibration source. Because the loader already has dedicated helpers for matrix and integer extraction, the implementation change is limited to the key paths passed into those helpers.

## Design

### Loader Behavior

`LoadFrontWideCameraModel()` will continue to:
- Load `T_car_lidar` from the LiDAR calibration file
- Invert `T_car_lidar` into `T_lidar_car`
- Load `T_car_cam` from the camera calibration file
- Invert `T_car_cam` into `T_cam_car`
- Load the camera intrinsic matrix into `K`
- Load `image_width` and `image_height`
- Reject non-positive image dimensions

The only behavior change is the source path inside `calib_camera_front_wide_to_car.json`:
- `T_car_cam` must come from `camera-front-wide-to-car.param.sensor_calib.data`
- `K` must come from `camera-front-wide.param.cam_matrix.data`
- `image_width` must come from `camera-front-wide.param.width`
- `image_height` must come from `camera-front-wide.param.height`

### Explicit Non-Goals

The loader must not:
- Read `camera-front-wide-undistort`
- Read `camera-front-wide-to-car-undistort`
- Check whether either `undistort` node exists
- Fall back to `undistort` nodes when the required non-`undistort` nodes are missing

If the required non-`undistort` nodes are absent, malformed, or have invalid dimensions, the function must fail exactly as it does today: log an error and return `false`.

### Test Updates

The calibration unit test fixture will be rewritten so the valid JSON case contains only:
- `camera-front-wide`
- `camera-front-wide-to-car`

The invalid-dimension fixture will also use only those same non-`undistort` nodes.

The test will continue to verify:
- Intrinsic matrix values are loaded correctly
- Camera and LiDAR extrinsics are loaded correctly
- Inverse transforms remain consistent
- Non-positive width or height causes load failure

## Error Handling

No new error-handling branches are introduced.

Expected failure cases remain:
- Camera calibration JSON cannot be opened
- Camera calibration JSON cannot be parsed
- Required non-`undistort` nodes are missing
- Required matrix or scalar values have the wrong shape or type
- `width <= 0` or `height <= 0`

The presence of `undistort` nodes elsewhere in the file has no effect on success or failure.

## Acceptance Criteria

The change is complete when:
- The loader reads front-wide intrinsics only from `camera-front-wide`
- The loader reads front-wide extrinsics only from `camera-front-wide-to-car`
- No code path reads or validates either `undistort` node
- The updated calibration unit test passes with fixtures that use only the non-`undistort` nodes
- The invalid-dimension test still fails as expected

## Files Affected

Modify:
- [camera_calibration.h](/workspace/Vision2PointFilter/src/projection/camera_calibration.h)
- [camera_calibration_test.cc](/workspace/Vision2PointFilter/tests/projection/camera_calibration_test.cc)
