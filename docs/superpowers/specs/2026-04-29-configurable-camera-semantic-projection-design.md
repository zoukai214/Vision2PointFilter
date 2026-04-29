# 可配置相机语义投影设计

## 背景

当前项目已经具备以下能力：

- 对原始 LiDAR 点云执行 deskew，并导出新的 PCD。
- 基于单路 `front_wide` 相机进行点云投影并输出可视化 PNG。
- 基于单路语义图为导出点云补充 `semantic_label` 字段。

当前实现的主要限制是：

- 投影和语义查询流程绑定在单路 `front_wide` 相机。
- 配置中使用单一路径字段，无法显式选择参与投影的相机集合。
- 输出投影图只能生成单路结果。

本次需要把“参与投影与语义融合的相机集合”改成显式配置，并让最终点云的 `semantic_label` 由配置中的多路相机按顺序融合得到。

## 目标

- 在配置文件中显式声明需要参与投影和语义查询的相机列表。
- 每个相机单独完成图像匹配、点云投影和 PNG 输出。
- 最终导出的点云只保留一个 `semantic_label` 字段。
- `semantic_label` 由配置相机按顺序进行多路语义融合得到。
- 不保留旧的单相机配置协议。

## 非目标

- 不修改 deskew 算法本身。
- 不引入投票融合、置信度融合或遮挡推理。
- 不在导出点云中增加 `semantic_label_<camera_name>` 等每相机字段。
- 不修改 CMake 编译选项。
- 不自动兼容旧配置中的 `projection.image_subdir`。

## 配置设计

### 新配置结构

`deskew_clip_export.projection` 改为以下结构：

```yaml
deskew_clip_export:
  projection:
    enabled: true
    image_root_subdir: images_seg_mask2former
    camera_names: [front_wide, back, left_front]
    output_subdir: projection
    max_time_diff_ms: 100.0
    point_radius_px: 2
    intensity_color_map: turbo
```

字段含义如下：

- `enabled`
  - 是否启用相机投影与语义融合。
- `image_root_subdir`
  - 语义图根目录，相对 `clip_root` 解析。
- `camera_names`
  - 参与投影和语义融合的相机列表。
  - 列表顺序同时定义语义融合顺序和投影处理顺序。
- `output_subdir`
  - 投影图输出根目录，相对 `output_dir` 解析。
- `max_time_diff_ms`
  - 每路相机图像与点云帧允许的最大时间差。
- `point_radius_px`
  - 投影渲染点半径。
- `intensity_color_map`
  - 投影渲染使用的颜色映射。

### 路径推导规则

给定相机名 `<camera_name>`：

- 语义图目录路径为：
  - `<clip_root>/<image_root_subdir>/<camera_name>`
- 相机标定文件路径为：
  - `<clip_root>/calib_extract/calib_camera_<camera_name>_to_car.json`
- 投影图输出目录路径为：
  - `<output_dir>/<output_subdir>/<camera_name>`

本次设计要求所有相机遵循统一命名规则，不支持为单个相机单独配置目录。

### 配置校验规则

当 `projection.enabled=true` 时，以下条件必须全部满足，否则启动失败：

- `image_root_subdir` 非空
- `camera_names` 字段存在
- `camera_names` 至少包含一个相机
- `camera_names` 中不存在重复项
- `output_subdir` 非空
- `max_time_diff_ms >= 0`
- `point_radius_px > 0`
- `intensity_color_map` 非空

同时，启动阶段需要对每个相机继续校验：

- 推导出的标定文件存在且可解析
- 推导出的语义图目录存在
- 图像索引能够成功建立

旧字段 `projection.image_subdir` 不再解析。如果配置仍然使用旧协议，应视为配置错误并失败。

## 总体方案

采用“显式相机列表驱动的多相机流程”方案：

- 将当前单路相机模型泛化为通用 `CameraModel`
- 启动时按照 `camera_names` 构建多相机上下文集合
- 每路上下文独立持有相机模型、图像索引和输出目录
- 每帧点云只 deskew 一次
- deskew 后的点在所有配置相机上逐路投影并尝试查询语义
- 每路投影图单独输出
- 最终点云只写入融合后的 `semantic_label`

该方案的核心原则是：相机集合和融合顺序完全由配置文件显式定义，业务代码不再内置固定 7 路相机顺序，也不保留 `front_wide` 特判。

## 数据结构设计

### 1. 通用相机模型

将当前 `FrontWideCameraModel` 泛化为 `CameraModel`，保留现有几何字段，并增加相机名：

- `camera_name`
- `T_lidar_car`
- `T_car_lidar`
- `T_car_cam`
- `T_cam_car`
- `K`
- `image_width`
- `image_height`

其中 `camera_name` 用于日志、路径关联和错误定位。

### 2. 多相机运行上下文

应用层为每个配置相机构建一个上下文对象，建议字段包括：

- `camera_model`
- `image_index`
- `output_dir`
- `render_config`
- `max_time_diff_ms`

应用主流程持有一个按 `camera_names` 顺序排列的上下文列表。顺序不可打乱，因为该顺序同时决定语义融合顺序。

### 3. 导出点类型

导出点类型继续只保留一个 `semantic_label` 字段：

- `x`
- `y`
- `z`
- `intensity`
- `ring`
- `point_time_offset`
- `semantic_label`

不新增每相机语义字段。

## 标定与图像加载设计

### 1. 相机标定加载

新增按相机名驱动的相机模型加载接口，例如：

- `LoadCameraModel(camera_name, lidar_top_to_car_path, camera_to_car_path, model)`

读取规则沿用当前单相机实现，只是将 key 从写死的 `front_wide` 改为按相机名拼接：

- 内参：`camera-<camera_key>.param.cam_matrix.data`
- 图像宽度：`camera-<camera_key>.param.width`
- 图像高度：`camera-<camera_key>.param.height`
- 相机到车体外参：`camera-<camera_key>-to-car.param.sensor_calib.data`
- LiDAR 到车体外参：`lidar-top-to-car.param.sensor_calib.data`

其中 `<camera_key>` 为将配置相机名中的下划线替换为中划线后的形式，例如：

- `front_wide -> front-wide`
- `left_front -> left-front`

### 2. 图像索引建立

每个相机都基于其独立语义图目录建立 `ImageIndex`。主流程启动时完成全部索引建立，任一路失败则整体失败。

### 3. 语义映射加载

灰度值到语义标签的映射仍然共享一份 `class_to_grayscale_mapping_panoptic.json`，只加载一次，在所有相机之间共享。

## 逐帧处理流程

对每个点云帧执行以下流程：

1. 读取原始点云并完成 deskew
2. 对每个配置相机查找最近语义图
3. 校验每路最近图像时间差不超过 `max_time_diff_ms`
4. 读取每路语义图，并校验为 `CV_8UC1`
5. 对 deskew 后点云中的每个点，按 `camera_names` 顺序逐路执行语义查询
6. 将融合后的结果写入导出点的 `semantic_label`
7. 保存带 `semantic_label` 的 PCD
8. 对每个相机分别渲染并保存一张投影 PNG

该流程中，点云 deskew 结果由所有相机共享，不重复计算。

## 语义融合规则

### 1. 单点处理规则

对每个点，按 `camera_names` 顺序依次处理：

1. 使用当前相机模型将点投影到该路语义图
2. 若投影失败，则继续下一路
3. 若像素越界，则继续下一路
4. 读取该像素灰度值并查询语义映射
5. 若灰度值未映射到语义标签，则继续下一路
6. 命中第一路有效标签后立即返回，并作为最终 `semantic_label`
7. 若所有相机都未命中，则 `semantic_label=-1`

### 2. 融合语义说明

本次“综合多个相机语义信息”的含义是：

- 在多个相机上分别尝试查询语义
- 按配置顺序选择第一路有效结果
- 不做投票、不做平均、不做冲突消解

因此相机列表顺序具有明确业务意义，应由配置显式控制。

## 投影 PNG 输出规则

每个相机的投影图都单独输出，不共享输出文件。

输出目录结构为：

```text
<output_dir>/<output_subdir>/<camera_name>/<timestamp>.png
```

例如：

```text
output/projection/front_wide/1710000000000.png
output/projection/back/1710000000000.png
output/projection/left_front/1710000000000.png
```

这样可以保证：

- 不同相机结果互不覆盖
- 输出目录具备清晰相机语义
- 增减相机时无需修改输出协议

## 错误处理

以下场景属于运行错误，应直接失败：

- 配置缺少 `image_root_subdir` 或 `camera_names`
- `camera_names` 为空或有重复
- 任一路相机标定文件不存在或解析失败
- 任一路图像目录不存在
- 任一路图像索引建立失败
- 任一路最近图像不存在
- 任一路最近图像时间差超过 `max_time_diff_ms`
- 任一路语义图无法读取
- 任一路语义图类型不是 `CV_8UC1`
- 语义映射文件不存在或解析失败

以下场景不应导致运行失败，只影响当前点的 `semantic_label`：

- 单个点在某一路投影失败
- 单个点投影后落在图像外
- 单个点命中灰度值但该灰度值未在映射表中定义
- 某一路对该点未命中有效语义，继续尝试后续相机

## 代码落点

建议修改范围如下：

- `src/projection/front_wide_projection_types.h`
  - 将单路相机模型泛化为 `CameraModel`
- `src/projection/camera_calibration.h`
  - 增加按相机名加载的通用标定接口
- `src/data_loader/gac_clip_root_loader.h/.cc`
  - 增加按相机名推导标定和图像目录的通用路径接口
- `src/projection/point_cloud_projector.h/.cc`
  - 将投影接口改为接收通用 `CameraModel`
- `src/projection/semantic_point_labeler.h/.cc`
  - 从单相机查询扩展为按有序相机列表执行逐路语义融合
- `src/application/deskew_clip_export.cc`
  - 解析新配置结构
  - 构建多相机上下文
  - 每帧加载多路语义图
  - 为每个点执行多相机语义融合
  - 为每个相机分别输出投影 PNG

## 测试设计

### 1. 配置解析测试

验证以下情况：

- `camera_names` 缺失时报错
- `camera_names` 为空时报错
- `camera_names` 有重复时报错
- `image_root_subdir` 缺失时报错
- 合法多相机配置能成功解析

### 2. 标定加载测试

验证以下情况：

- 不同相机名能正确推导 JSON key 并加载标定
- 下划线转中划线规则正确
- 不存在的相机标定文件会被拒绝

### 3. 多相机语义融合测试

构造小尺寸相机模型、测试图像和测试点，验证：

- 第一台相机命中时直接返回其标签
- 第一台相机未命中时能回退到第二台相机
- 所有相机都未命中时返回 `-1`
- 图像类型非法时返回失败

### 4. 应用层输出测试

验证以下情况：

- 多相机配置时会为每个相机创建单独输出目录
- 每个相机都分别生成对应投影 PNG
- 导出 PCD 只包含一个融合后的 `semantic_label`

## 风险与控制

### 风险 1：接口仍残留单相机命名

如果只在应用层堆叠多相机逻辑，而不抽象投影和标定接口，会继续扩大 `front_wide` 命名和语义不一致的问题。

控制策略：

- 统一将底层接口泛化为按相机名驱动的通用模型和函数。

### 风险 2：配置顺序与融合顺序不一致

如果实现阶段用无序容器管理相机，可能导致融合结果不稳定。

控制策略：

- 主流程使用保序容器保存 `camera_names` 和相机上下文。

### 风险 3：多路图像匹配失败导致结果不完整

如果某一路图像不存在或时间差超限，而流程继续运行，最终 `semantic_label` 会悄悄退化。

控制策略：

- 对每路相机采用严格校验策略，任一路失败直接终止当前运行。

## 验收标准

- 配置文件可以显式声明参与投影的相机列表
- 每路相机都能根据统一命名规则加载标定和语义图
- 每个相机单独输出投影 PNG
- 导出点云只包含一个 `semantic_label`
- `semantic_label` 按 `camera_names` 顺序融合多路语义结果
- 不再依赖旧的单相机配置字段
- 新增对应单元测试并通过
