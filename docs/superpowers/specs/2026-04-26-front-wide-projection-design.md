# Front Wide Projection Design

## Goal

Extend the existing single-frame deskew export flow so that each deskewed LiDAR frame is projected onto the undistorted `front_wide` image and exported as a visualization image. This stage is for geometric validation only. It does not write per-point semantic labels back into the point cloud yet.

## Scope

In scope:
- Reuse the current `deskew_clip_export` executable and per-frame processing loop.
- After each frame is deskewed, project the in-memory deskewed cloud onto `images_seg_mask2former/front_wide`.
- Match each point cloud frame to the nearest image by filename timestamp.
- Fail fast if the nearest image is more than `100 ms` away.
- Use the undistorted front-wide camera model for projection.
- Render projected points on top of the image using intensity-based pseudo color.
- Export one projection image per successful deskewed frame.

Out of scope:
- Writing semantic class IDs or colors back into the point cloud.
- Supporting multiple cameras.
- Occlusion reasoning or z-buffer visibility resolution.
- Adding a second executable or a separate offline projection pipeline.

## Existing Context

The repository currently contains a single application entry point at [src/application/deskew_clip_export.cc](/workspace/Vision2PointFilter/src/application/deskew_clip_export.cc). It reads raw top LiDAR PCD files, interpolates motion from `IE_post_traj.asc`, deskews each frame, and writes output PCDs to `output/deskew_pcd`.

The target dataset layout already contains:
- `lidar_raw/top/*.pcd`
- `images_seg_mask2former/front_wide/*.png`
- `calib_extract/calib_lidar_top_to_car.json`
- `calib_extract/calib_camera_front_wide_to_car.json`

The `front_wide` images are already undistorted, so projection must use the undistorted camera intrinsics and a standard pinhole model with no additional distortion step.

## Recommended Approach

Keep `deskew_clip_export` as the only executable and add a dedicated projection module under `src/projection/`. The application remains responsible for orchestration, while the new module owns calibration parsing, image indexing, timestamp matching, projection, and rendering.

This keeps the current flow and avoids redundant PCD reloads, while preventing the application entry point from absorbing all projection details.

## Processing Flow

For each selected LiDAR frame:

1. Load the raw PCD and deskew it using the existing motion compensation logic.
2. Save the deskewed cloud to `deskew_pcd/<stem>.pcd`.
3. Parse the point cloud timestamp from the PCD filename stem.
4. Find the nearest `front_wide` image by timestamp.
5. Compute the absolute timestamp difference.
6. If the difference is greater than `100 ms`, log a detailed error and terminate the program.
7. Load the matched image.
8. Project the in-memory deskewed cloud into the undistorted front-wide camera.
9. Draw valid projected points on top of the image using intensity-based pseudo color.
10. Save the rendered overlay to `projection_front_wide/<stem>.png`.

The point cloud stem and projection image stem must remain identical so outputs can be paired trivially.

## Module Design

### `camera_calibration`

Responsibility:
- Read `calib_lidar_top_to_car.json`
- Read `calib_camera_front_wide_to_car.json`
- Extract:
  - LiDAR-to-car extrinsic
  - front-wide undistorted camera intrinsic
  - front-wide camera-to-car extrinsic source
  - image width and height

Output model:
- `T_car_lidar`
- `T_car_cam` from the undistorted front-wide camera entry
- `T_cam_car = inverse(T_car_cam)`
- `K_undistort`
- `image_width`
- `image_height`

This module must not expose raw JSON parsing details to the application layer.

### `image_index`

Responsibility:
- Scan `images_seg_mask2former/front_wide/*.png`
- Extract timestamps from filenames
- Keep an ordered index
- Return the nearest image to a given point cloud timestamp
- Return the match time difference

This module centralizes filename parsing and nearest-neighbor matching so time validation is implemented in one place.

### `point_cloud_projector`

Responsibility:
- Accept a deskewed point cloud, matched image, and calibrated camera model
- Transform points from LiDAR frame to camera frame
- Project valid points into image coordinates
- Convert intensity to overlay color
- Draw points on the image
- Return the rendered visualization image

The application should treat this as a high-level projection service instead of mixing math and rendering into `main()`.

## Coordinate and Projection Model

The transform chain is:

- `p_car = T_car_lidar * p_lidar`
- `p_cam = T_cam_car * p_car`

Where:
- `T_car_lidar` comes directly from `calib_lidar_top_to_car.json`
- `T_car_cam` comes from the front-wide camera extrinsic entry in `calib_camera_front_wide_to_car.json`
- `T_cam_car` is computed by inversion

Projection rules:
- Only keep points with `p_cam.z > 0`
- Apply standard pinhole projection with the undistorted intrinsic matrix
- Convert normalized image coordinates to pixel coordinates
- Keep only points whose pixels fall inside `[0, width)` and `[0, height)`

No distortion model is applied during projection because the target images are already undistorted.

## Rendering Rules

The base image is the matched `images_seg_mask2former/front_wide` PNG.

Projected points are rendered as filled circles with a configurable pixel radius. Point color is derived from LiDAR intensity using a fixed pseudo-color ramp intended for visual contrast, such as blue to cyan to yellow to red.

Normalization rules:
- Compute the intensity range from valid projected points in the current frame
- If the range is non-degenerate, linearly normalize into `[0, 1]`
- If the range collapses to a single value, render all points with one fallback color

This visualization is strictly for inspection and calibration validation. It does not need to preserve the semantic meaning of the image pixels beyond using them as a background.

## Configuration Changes

Extend [config/deskew_clip_export.yaml](/workspace/Vision2PointFilter/config/deskew_clip_export.yaml) with a `projection` section:

```yaml
deskew_clip_export:
  projection:
    enabled: true
    image_subdir: images_seg_mask2former/front_wide
    output_subdir: projection_front_wide
    max_time_diff_ms: 100.0
    point_radius_px: 2
    intensity_color_map: turbo
```

Notes:
- The camera target remains fixed to `front_wide` for this phase.
- `enabled` allows projection to be turned off without removing the new code path.
- `image_subdir` and `output_subdir` should remain configurable rather than hard-coded.
- `max_time_diff_ms` defaults to `100.0` and is enforced as a hard failure threshold.

## Output Layout

For a point cloud stem such as `1758521322500000000_0`:

- Deskew output: `output/deskew_pcd/1758521322500000000_0.pcd`
- Projection output: `output/projection_front_wide/1758521322500000000_0.png`

If the deskew stage succeeds, the projection stage is expected to produce a corresponding image unless the program terminates on a hard error.

## Error Handling

The program must fail immediately on these conditions:
- Missing front-wide image directory
- Missing required calibration files
- Missing undistorted front-wide intrinsic or extrinsic entries
- Failed image decode
- No image found near a point cloud timestamp
- Nearest image timestamp difference greater than `100 ms`

The error log for time mismatch must include:
- Point cloud path
- Matched image path
- Point cloud timestamp
- Image timestamp
- Absolute time difference in milliseconds

The following conditions are frame-local and non-fatal unless they reflect a broader I/O failure:
- Points behind the camera are skipped
- Points outside image bounds are skipped
- Zero valid projected points still produces an output image, but logs the valid point count as `0`

## Dependencies

Rendering image overlays requires image I/O and drawing support. The cleanest option is to add OpenCV as a build dependency for:
- PNG image loading
- Image writing
- Circle drawing

If OpenCV is introduced, its usage should remain isolated to the projection module so the existing deskew code path does not become image-library aware.

## Testing and Acceptance

### Startup Validation

The program must verify before processing:
- The front-wide image directory exists and is not empty
- Both calibration files exist
- The undistorted camera model can be parsed
- The projection output directory can be created

### Per-Frame Validation

For each processed frame:
- A nearest image match is computed from timestamps
- The match delta is logged or otherwise observable during execution
- A delta above `100 ms` causes immediate termination

### Geometric Validation

Manual spot checks on several output images should show:
- Road and scene structures in approximately correct image regions
- No global left-right inversion
- No global up-down inversion
- No large systematic offset indicating a broken extrinsic chain
- No edge-wide warping consistent with using the wrong intrinsic model

### Output Validation

Successful frames must produce both:
- `deskew_pcd/<stem>.pcd`
- `projection_front_wide/<stem>.png`

The counts of successful projection images and successful deskewed PCD files should match in a clean run.

### Boundary Validation

The implementation must explicitly handle:
- Points behind the camera
- Points outside image bounds
- Frames with zero valid projected points
- Missing images
- Images whose timestamps violate the `100 ms` threshold

## Future Extensions

This design intentionally leaves room for later work without expanding the current scope:
- Projecting to additional cameras
- Writing semantic values back into each point record
- Multi-camera best-view selection
- Visibility reasoning with z-buffering

These are postponed so the current phase remains focused on producing correct front-wide projection overlays after deskew.
