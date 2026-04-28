# Front-Wide Calibration No-Undistort Design

## Background

`LoadFrontWideCameraModel` 目前仍从前广角标定 JSON 中读取以下旧字段：

- `camera-front-wide-undistort.param.cam_matrix.data`
- `camera-front-wide-undistort.param.width`
- `camera-front-wide-undistort.param.height`
- `camera-front-wide-to-car-undistort.param.sensor_calib.data`

这与当前需求不一致。当前需求是只使用原始前广角相机内参与相机到车体外参，不加载去畸变参数，也不审查畸变相关字段。

## Goal

将前广角标定加载逻辑切换为只依赖以下字段：

- `camera-front-wide.param.cam_matrix.data`
- `camera-front-wide.param.width`
- `camera-front-wide.param.height`
- `camera-front-wide-to-car.param.sensor_calib.data`

JSON 中其它无关字段，包括任意畸变参数和 `*-undistort` 字段，应被完全忽略。

## Scope

本次只修改前广角标定加载器及其单元测试：

- `src/projection/camera_calibration.h`
- `tests/projection/camera_calibration_test.cc`

不修改构建配置，不调整其它投影模块，不做全局标定格式迁移。

## Design

### Loader Behavior

`LoadFrontWideCameraModel` 保持现有接口不变，继续接收：

- `lidar_top_to_car_path`
- `camera_front_wide_to_car_path`
- `FrontWideCameraModel* model`

实现改为：

1. 继续从 lidar 标定文件加载 `lidar-top-to-car.param.sensor_calib.data`
2. 从相机标定 JSON 加载 `camera-front-wide-to-car.param.sensor_calib.data`
3. 从同一 JSON 加载 `camera-front-wide.param.cam_matrix.data`
4. 从同一 JSON 加载 `camera-front-wide.param.width`
5. 从同一 JSON 加载 `camera-front-wide.param.height`

### Validation

只对被业务使用的字段做校验：

- 外参矩阵必须成功读取
- 内参矩阵必须为 3x3
- `width` 和 `height` 必须为正整数

以下内容不做读取，也不做审查：

- 畸变系数
- `camera-front-wide-undistort`
- `camera-front-wide-to-car-undistort`
- 任意其它非必需字段

### Error Handling

如果缺少必需 key，或必需字段类型错误，保持当前失败返回行为：

- 输出现有风格的错误日志
- `LoadFrontWideCameraModel` 返回 `false`

如果 JSON 中存在额外字段，但必需字段完整且合法，则加载成功。

## Testing

更新现有单元测试覆盖以下场景：

1. 仅提供 `camera-front-wide` 和 `camera-front-wide-to-car` 时加载成功
2. 即使 JSON 中附带 `*-undistort` 或畸变字段，也必须忽略，不影响加载结果
3. 当 `camera-front-wide.width` 或 `camera-front-wide.height` 非正时加载失败

## Non-Goals

- 不兼容回退到 `*-undistort`
- 不解析畸变模型或畸变系数
- 不修改其它模块对 `FrontWideCameraModel` 的使用方式
