# Deskew Global Frame ENU UTM Design

## Background

当前 `deskew_clip_export` 运动补偿主流程存在两个问题：

1. GNSS 到 LiDAR 标定文件路径固定为 `calib_gnss_to_lidar_top_ENU.json`
2. ASC 轨迹解析后的世界坐标始终按 UTM 使用，没有像 `icv_mapping` 那样在 `enu` / `utm` 模式下做不同处理

用户要求保留现有 ENU 方案，同时增加 UTM 方案，并通过配置显式选择。两种模式都读取同一个外参文件：

- `calib_gnss_to_lidar_top_ENU.json`

两种模式的区别只体现在轨迹坐标与姿态转换语义上，不体现在 GNSS 到 LiDAR 外参文件选择上。

## Goal

让 `deskew_clip_export` 支持与 `icv_mapping` 对齐的两种全局坐标系模式：

- `enu`
- `utm`

两种模式下，运动补偿都必须可运行，但全局位姿转换逻辑按模式严格分支。

## Scope

本次修改范围限定为：

- `config/deskew_clip_export.yaml`
- `src/application/deskew_clip_export.cc`
- `src/data_loader/gac_clip_root_loader.h`
- `src/data_loader/gac_clip_root_loader.cc`
- `src/pose_parse/inspvax_asc_parser.h`
- `src/pose_parse/inspvax_asc_parser.cc`
- 新增对应测试

不修改现有构建选项，不改动投影算法本身，不引入自动兼容策略。

## Design

### 1. Config

在 `deskew_clip_export` 配置中增加：

- `pose.coord_frame: enu | utm`

默认值使用 `enu`，保持现有数据组织的默认预期。

配置校验规则：

- 仅允许 `enu` 或 `utm`
- 其它值直接报错退出

### 2. Calibration File Selection

两种模式统一读取：

- `GnssToLidarTopEnuPath()`
- 对应文件 `calib_extract/calib_gnss_to_lidar_top_ENU.json`

`LoadGnssToLidarTop(...)` 作为通用外参加载函数，继续按相同 JSON key 读取。

若该文件不存在或格式错误，直接失败。

### 3. Pose Semantics

对齐 `icv_mapping` 的语义：

- `utm` 模式：
  - ASC 解析结果继续使用 UTM 位置
  - yaw 保留 UTM convergence 补偿
- `enu` 模式：
  - 关闭 UTM convergence 补偿
  - 以首个 PCD 时间戳之前的 origin sample 作为本地 ENU 原点
  - 将所有 `InspvaxSample` 的位置从经纬高转换为 local ENU
  - 再对样本整体做一次 origin 重心化，保证原点附近数值稳定

为兼容当前字段命名，`InspvaxSample::utm_m` 继续作为“当前导出世界坐标”的容器：

- `utm` 模式下其数值语义为 UTM
- `enu` 模式下其数值语义为 local ENU

### 4. Local ENU Conversion

在 `InspvaxAscParser` 中增加最小必要接口：

- `ToLocalEnu(lat, lon, alt, origin_llh, enu_out, error)`

实现路径：

1. WGS84 经纬高转 ECEF
2. 当前点与原点做 ECEF 差分
3. ECEF 差分转 ENU

该接口只负责位置转换，不改姿态。

### 5. Deskew Flow

主流程保持单一补偿算法不变，仍然统一构造：

- `T_w_gnss`
- `T_w_lidar`

区别只在于：

- `w` 的语义由配置决定，是 `ENU` 或 `UTM`
- `T_gnss_lidar` 始终来自 `calib_gnss_to_lidar_top_ENU.json`
- `samples` 的平移值在 `enu` 模式下预先改写为 local ENU
- parser 的 `apply_utm_convergence` 在 `enu` 模式下关闭

### 6. Error Handling

严格失败场景：

- `pose.coord_frame` 非法
- 缺少 `calib_gnss_to_lidar_top_ENU.json`
- ENU 转换失败
- origin sample 无法确定

日志需要明确打印当前模式与正在读取的标定文件路径。

## Testing

新增聚焦测试覆盖：

1. 配置解析接受 `enu` / `utm`，拒绝非法值
2. `enu` / `utm` 模式都只读取 `calib_gnss_to_lidar_top_ENU.json`
3. `enu` / `utm` 模式不会因为配置不同而切换外参文件
4. `enu` 模式关闭 UTM convergence，`utm` 模式开启
5. `enu` 模式会将样本位置改写为 local ENU
6. 不存在任何自动回退行为

## Non-Goals

- 不支持通过模式切换 GNSS 到 LiDAR 外参文件
- 不修改点云投影坐标定义
- 不复刻 `icv_mapping` 中与 label、ground、map_origin、override extrinsic 相关的完整逻辑
