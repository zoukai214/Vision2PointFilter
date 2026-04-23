# Segment Projection Deskew Tool

这个项目实现了和 `icv_mapping` 输入组织方式一致的单帧点云运动补偿。

## 输入目录约定

给定一个 `clip_root`，程序按下面的固定层级读取数据：

- `calib_extract/calib_gnss_to_lidar_top_ENU.json`
- `ego_raw/IE_post_traj.asc`
- `lidar_raw/top/*.pcd`

其中点云类型与 `icv_mapping` 保持一致，要求 PCD 中包含：

- `x`
- `y`
- `z`
- `intensity`
- `ring`
- `point_time_offset`

`point_time_offset` 的含义与 `icv_mapping` 一致，为相对当前 PCD 文件时间戳的微秒偏移。

## 输出

输出目录默认为 `<clip_root>/output/deskew_pcd/`。

输出文件名与输入 PCD 文件名保持一致，例如：

- 输入：`lidar_raw/top/1758521322500000000_0.pcd`
- 输出：`output/deskew_pcd/1758521322500000000_0.pcd`

## 构建

```bash
python3 scripts/run_deskew_clip.py --build-only
```

## 运行

```bash
python3 scripts/run_deskew_clip.py /path/to/clip_root
```

或者指定输出目录：

```bash
python3 scripts/run_deskew_clip.py /path/to/clip_root /path/to/output_dir
```

程序读取配置文件：

- [config/deskew_clip_export.yaml](/workspace/segment_projection/config/deskew_clip_export.yaml)

可调参数包括：

- `frame_stride`
- `deskew.max_range_m`
- `deskew.interp_max_gap_ms`
- `output.subdir`

## 当前实现范围

当前项目只做单帧点云运动补偿和结果导出，不包含 `icv_mapping` 中的：

- label 过滤
- 地面估计
- 融合建图
- LAZ 导出
