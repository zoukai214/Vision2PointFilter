# 原始畸变图投影支持设计

## 背景

当前项目默认输入图片为去畸变图，投影时按针孔模型直接使用内参矩阵进行像素投影。

现需新增对原始未去畸变图的支持。输入相机标定文件参考：

- `/workspace/GACRT026_1758521322/calib_extract/calib_camera_front_wide_to_car.json`

该标定文件同时包含：

- `camera-front-wide`：原始相机内参、畸变系数
- `camera-front-wide-to-car`：原始相机外参
- `camera-front-wide-undistort`：去畸变图对应内参
- `camera-front-wide-to-car-undistort`：去畸变图对应外参

本次需求已明确：

- 默认输入仍是去畸变图
- 新增支持原始未去畸变图
- 标定读取时始终使用 `camera-front-wide` 中的内参与畸变系数
- 不再读取 `camera-front-wide-undistort` 参数
- 当前输入图像类型通过配置文件显式指定
- 当输入为原始未去畸变图时，根据畸变系数中非零项个数判断使用 4 参数 fisheye 模型还是 8 参数畸变模型

## 目标

实现一套兼容现有流程的投影扩展方案，使系统既能处理去畸变图，也能处理原始未去畸变图，并保持旧配置行为不变。

## 非目标

- 不修改现有目录组织方式
- 不引入自动识别输入图像类型的隐式逻辑
- 不修改 CMake 编译选项
- 不扩展为按相机分别配置不同图像类型
- 不依赖 `camera-*-undistort` 节点进行内参切换

## 方案对比

### 方案一：在 projection 下新增统一配置项 `image_model`

在投影配置中新增 `image_model: undistorted | raw`，用于明确当前输入图像类型。

优点：

- 语义直接，配置含义清晰
- 与当前 `projection` 结构兼容，侵入最小
- 旧配置可通过默认值保持原行为

缺点：

- 当前同一任务内无法为不同相机配置不同图像类型

### 方案二：按相机配置输入图像类型

例如新增 `camera_image_models` 映射，为不同相机分别配置 `raw` 或 `undistorted`。

优点：

- 灵活性更高

缺点：

- 明显增加配置复杂度与校验复杂度
- 当前需求没有必要引入该复杂度

### 方案三：根据目录名或路径规则推断输入图像类型

例如从 `image_root_subdir` 自动推断是原图还是去畸变图。

优点：

- 配置改动最少

缺点：

- 规则脆弱，目录名变化即失效
- 语义不显式，难以维护

## 选型结论

采用方案一。

原因：

- 输入图像类型应由配置显式表达，而不是由目录名或畸变系数间接推断
- 现阶段不需要按相机单独配置，统一配置更符合最小改动原则
- 能完整兼容现有配置与现有调用流程

## 配置设计

### 新增字段

在 `deskew_clip_export.projection` 下新增：

```yaml
image_model: undistorted
```

可选值：

- `undistorted`：输入图像为去畸变图
- `raw`：输入图像为原始未去畸变图

默认值：

- `undistorted`

### 配置示例

去畸变图输入：

```yaml
deskew_clip_export:
  projection:
    enabled: true
    image_root_subdir: images_seg_mask2former
    image_model: undistorted
    camera_names: [front_wide]
    output_subdir: projection
    max_time_diff_ms: 100.0
    point_radius_px: 2
    intensity_color_map: turbo
```

原始未去畸变图输入：

```yaml
deskew_clip_export:
  projection:
    enabled: true
    image_root_subdir: images_seg_mask2former
    image_model: raw
    camera_names: [front_wide]
    output_subdir: projection
    max_time_diff_ms: 100.0
    point_radius_px: 2
    intensity_color_map: turbo
```

### 配置校验

解析配置时执行以下规则：

- 未配置 `image_model` 时，使用默认值 `undistorted`
- `image_model` 必须为 `undistorted` 或 `raw`
- 非法值直接报错并终止加载

## 标定读取设计

### 读取原则

相机标定始终从原始相机节点读取，不根据输入图像类型切换到 `*-undistort` 节点。

对于 `front_wide`，具体读取规则如下：

- 内参矩阵：`camera-front-wide.param.cam_matrix.data`
- 畸变系数：`camera-front-wide.param.cam_dist.data`
- 图像宽高：`camera-front-wide.param.width`、`camera-front-wide.param.height`
- 外参矩阵：`camera-front-wide-to-car.param.sensor_calib.data`

不读取：

- `camera-front-wide-undistort`
- `camera-front-wide-to-car-undistort`

### 数据模型扩展

当前 `CameraModel` 仅保存：

- 相机名
- 内外参矩阵
- 图像宽高

本次需要扩展保存：

- 畸变系数数组
- 原始图像投影所使用的畸变模型类型

建议新增的模型类型枚举语义：

- `kUndistortedPinhole`
- `kFisheye4`
- `kDistorted8`

其中：

- `kUndistortedPinhole` 不是从标定自动推导，而是运行时当配置指定 `image_model=undistorted` 时采用的投影路径
- `kFisheye4` 与 `kDistorted8` 用于原始图像投影路径

### 畸变模型判定规则

当配置指定输入图像为 `raw` 时，使用 `cam_dist.data` 的非零项个数判断投影模型：

- 非零项个数小于等于 4：按 4 参数 fisheye 模型投影
- 非零项个数大于 4：按 8 参数畸变模型投影

补充规则：

- 只统计有效非零项，尾部补零不参与模型升级判断
- 4 参数模型仅使用前 4 个有效系数
- 8 参数模型仅使用前 8 个有效系数
- 若 `cam_dist.data` 缺失，则按错误处理，拒绝进入 raw 投影流程
- 若所有畸变系数均为 0，则归入 4 参数 fisheye 分支处理，等价于零畸变 fisheye 参数集合

## 投影行为设计

### 总体行为

运行时将“输入图像类型判断”和“原始图像畸变模型判断”拆开处理：

1. 由配置项 `image_model` 判断当前输入是 `undistorted` 还是 `raw`
2. 仅当 `image_model=raw` 时，再根据 `cam_dist.data` 的非零项个数决定走 `kFisheye4` 还是 `kDistorted8`

### 去畸变图投影路径

当 `image_model=undistorted` 时：

- 继续使用现有针孔投影方式
- 使用 `camera-front-wide` 中的 `cam_matrix`
- 忽略畸变系数
- 保持当前像素落点逻辑不变

该路径的目的不是还原标定语义，而是保持当前系统对去畸变图输入的既有行为不回归。

### 原始未去畸变图投影路径

当 `image_model=raw` 时：

- 输入点先变换到相机坐标系
- 根据 `cam_dist.data` 判定模型类型
- 若为 `kFisheye4`，按 4 参数 fisheye 模型完成投影
- 若为 `kDistorted8`，按 8 参数畸变模型完成投影
- 输出像素坐标仍需执行有限值与图像边界校验

### 错误处理

以下场景直接返回失败并记录日志：

- `raw` 模式下缺少 `cam_dist`
- `cam_dist` 结构格式非法
- 宽高非法
- 投影结果非有限值
- 输入图像尺寸与标定尺寸不一致
- 配置项 `image_model` 非法

## 代码落点

### 配置层

文件：

- `src/application/deskew_clip_export_config.h`
- `src/application/deskew_clip_export_config.cc`

职责：

- 新增 `image_model` 字段
- 增加字符串到枚举或语义值的解析逻辑
- 做默认值和合法性校验

### 标定层

文件：

- `src/projection/front_wide_projection_types.h`
- `src/projection/camera_calibration.h`

职责：

- 在 `CameraModel` 中新增畸变参数存储
- 在 `LoadCameraModel` 中读取 `cam_dist.data`
- 基于非零项个数推导原始图像投影模型
- 始终从 `camera-xxx` 读取内参，不切换到 `camera-xxx-undistort`

### 投影层

文件：

- `src/projection/point_cloud_projector.h`
- `src/projection/point_cloud_projector.cc`
- `src/application/deskew_clip_export.cc`

职责：

- 将当前投影流程扩展为 `undistorted` 和 `raw` 两条路径
- 在主流程中把配置项传入投影逻辑
- 尽量保持现有对外接口稳定，减少对调用侧的连锁修改

## 测试设计

### 配置测试

文件：

- `tests/application/deskew_clip_export_config_test.cc`

新增覆盖：

- 未设置 `image_model` 时默认值为 `undistorted`
- `image_model=undistorted` 可正常解析
- `image_model=raw` 可正常解析
- 非法值报错

### 标定测试

文件：

- `tests/projection/camera_calibration_test.cc`

新增覆盖：

- 正常读取 `camera-front-wide.param.cam_dist.data`
- 不读取 `camera-front-wide-undistort` 内参
- 非零项个数小于等于 4 时识别为 4 参数 fisheye
- 非零项个数大于 4 时识别为 8 参数畸变模型
- 缺失 `cam_dist` 时在 raw 场景下可被上层正确拦截或返回错误

### 投影测试

文件：

- `tests/projection/point_cloud_projector_test.cc`

新增覆盖：

- `undistorted` 路径保持现有投影结果
- `raw + fisheye4` 路径可得到合理像素输出
- `raw + distorted8` 路径可得到合理像素输出
- 图像尺寸不匹配时返回失败
- 越界点与相机后方点继续被过滤

## 兼容性

- 旧配置不包含 `image_model` 时，行为保持与当前一致
- 当前默认输入去畸变图的流程不应发生行为回归
- 仅当显式配置 `image_model=raw` 时才启用原始图像畸变投影逻辑

## 风险与约束

### 风险

- 原始图像畸变模型公式实现若与标定生产链路不一致，会导致像素偏移
- 仅按非零项个数判断模型类型依赖上游标定文件格式稳定
- 若部分相机未来存在不同格式的 `cam_dist`，当前规则可能需要扩展

### 约束

- 当前方案按统一配置项控制整批图像类型
- 不支持同一任务中不同相机混用 raw 和 undistorted
- 不引入新的外部依赖

## 实施摘要

本次实现将通过新增 `projection.image_model` 配置项，明确区分去畸变图与原始未去畸变图输入；标定读取始终固定到 `camera-xxx` 原始节点；raw 路径下依据 `cam_dist.data` 非零项数量在 4 参数 fisheye 与 8 参数畸变模型之间切换；undistorted 路径继续保持现有针孔投影行为，以确保兼容现有流程。
