# 点云分类字段替换设计

## 背景

当前项目已经能够基于语义分割图为导出点云补充 `semantic_label` 字段。

- 点云导出字段定义位于 `src/data_loader/semantic_labeled_point.h`
- 导出流程位于 `src/application/deskew_clip_export.cc`
- 灰度值到语义编号的映射位于 `src/projection/semantic_label_mapping.cc`

现状中，分割图像素灰度值会先映射到 `contiguous_id`，再直接写入导出点云的 `semantic_label` 字段。

本次需求不再直接输出 `semantic_label`。新的导出字段需要命名为 `classification`，并且字段值不再是原始 `contiguous_id`，而是基于 `contiguous_id` 归并后的四类结果：`-1`、`0`、`1`、`2`。

## 目标

- 将导出点云字段名从 `semantic_label` 改为 `classification`
- 保持现有“灰度值 -> contiguous_id”的映射逻辑不变
- 在导出阶段新增“`contiguous_id` -> `classification`”归并逻辑
- 未命中语义时继续输出 `-1`
- 不改变点云点数，不引入删点行为

## 非目标

- 不修改 `class_to_grayscale_mapping_panoptic.json` 的结构和内容
- 不改变灰度图加载、相机投影、多相机融合的现有流程
- 不在本次改动中保留额外的导出字段，如同时输出 `semantic_label` 和 `classification`
- 不引入新的配置项来动态配置分类归并规则

## 总体方案

采用“保留现有语义查找链路，在导出前增加一层固定分类映射”的方案。

处理流程保持如下：

1. 像素灰度值按现有逻辑映射到 `contiguous_id`
2. 根据固定归并规则将 `contiguous_id` 转换为 `classification`
3. 将结果写入导出点云的新字段 `classification`

这样做的原因是：

- 改动范围最小，不需要重写现有语义映射模块的职责
- 可以最大程度复用已经验证过的 `gray_value -> contiguous_id` 逻辑
- 只改变最终导出语义，避免扩大接口和测试影响面

## 分类规则

本次 `classification` 只包含四个取值：`-1`、`0`、`1`、`2`。

归并规则固定如下。

### `classification = 1`

当 `contiguous_id` 属于以下集合时，输出 `classification = 1`：

`7, 8, 9, 10, 11, 12, 13, 14, 15, 23, 24, 26, 28, 29`

### `classification = 0`

当 `contiguous_id` 属于以下集合时，输出 `classification = 0`：

`2, 3, 4, 5, 6, 16, 17, 18, 25, 27, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51`

### `classification = 2`

当 `contiguous_id` 属于以下集合时，输出 `classification = 2`：

`0, 1, 19, 20, 21, 22, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64`

### `classification = -1`

以下情况统一输出 `classification = -1`：

- 原语义查找未命中，当前内部值为 `-1`
- 点无法投影到有效像素
- 投影落在图像外
- 灰度值无法映射到合法 `contiguous_id`

## 模块设计

### 1. 导出点类型

现有导出点类型继续保留结构布局，但字段名从 `semantic_label` 改为 `classification`。

字段集合变为：

- `x`
- `y`
- `z`
- `intensity`
- `ring`
- `point_time_offset`
- `classification`

要求：

- `classification` 类型仍为 `int`
- 默认值为 `-1`
- PCL 注册字段名同步改为 `classification`

这样最终保存的 PCD 字段名会直接变成 `classification`。

### 2. 分类归并模块

新增一个轻量的分类归并逻辑，用于把单个 `contiguous_id` 映射成 `classification`。

建议实现特征：

- 输入为 `int contiguous_id`
- 输出为 `int classification`
- 对 `-1` 直接返回 `-1`
- 对规则覆盖范围内的 `contiguous_id` 返回对应类别
- 如果出现规则外的 `contiguous_id`，默认返回 `-1`

这里不需要新的配置文件或运行时加载逻辑，直接使用代码内固定规则即可。

### 3. 导出流程调整

在 `deskew_clip_export` 中维持现有查找流程：

1. 为每个点执行多相机语义查找
2. 得到中间结果 `contiguous_id`，未命中则为 `-1`
3. 对该中间结果执行分类归并
4. 将归并结果写入导出点的 `classification`
5. 保存 PCD

为了保持代码清晰，建议不要让 `LookupSemanticLabelForPointMultiCamera` 直接承担分类归并职责。它继续负责查找原始语义编号即可，分类转换放在导出层完成。

## 错误处理

以下行为保持当前策略，不新增流程级失败条件：

- 单点投影失败
- 单点落图像外
- 单点灰度值未命中映射表

这些情况最终都只会使当前点的 `classification = -1`。

以下行为也不新增特殊失败逻辑，而是通过兜底方式处理：

- 归并逻辑收到规则外的 `contiguous_id`

该场景视为未知类别，直接输出 `classification = -1`。

现有流程已经定义的启动失败条件保持不变，例如：

- 语义映射 JSON 无法读取
- 映射 JSON 格式非法
- 语义图无法读取
- 语义图不是单通道灰度图

## 代码落点

建议修改范围如下：

- `src/data_loader/semantic_labeled_point.h`
  - 将字段 `semantic_label` 改为 `classification`
  - 更新 PCL 字段注册名
- `src/application/deskew_clip_export.cc`
  - 保留现有语义查找
  - 增加 `contiguous_id -> classification` 归并步骤
  - 导出时写入 `classification`
- `src/projection/` 或 `src/application/`
  - 新增一个小型辅助函数或模块，用于分类归并

命名上允许内部继续使用 `semantic label` 来描述中间 `contiguous_id`，但导出结构和最终 PCD 应统一使用 `classification`。

## 测试设计

本次至少需要覆盖以下两类测试。

### 1. 分类归并测试

验证内容包括：

- `-1` 能映射到 `-1`
- 每个分类集合中挑选若干代表值，验证能映射到正确的 `classification`
- 不在规则内的 `contiguous_id` 会映射到 `-1`

该测试应独立验证分类归并规则本身，避免把责任混入图像投影测试。

### 2. 导出链路测试

验证内容包括：

- 导出点类型字段名已经从 `semantic_label` 改为 `classification`
- 语义查找命中的点，最终导出值是归并后的 `classification`
- 原本会输出 `semantic_label = -1` 的点，最终导出 `classification = -1`

如果现有测试只覆盖到语义查找模块，则本次需要补充对导出层归并逻辑的验证。

## 风险与控制

### 风险 1：字段名改动影响下游读取

下游如果有脚本或工具明确依赖 `semantic_label` 字段名，本次会产生兼容性变化。

控制策略：

- 在设计和最终交付说明中明确这是一个导出字段改名
- 不保留双字段，避免新旧语义并存导致歧义

### 风险 2：分类规则遗漏

如果固定规则没有覆盖某些合法 `contiguous_id`，这些点会被归并为 `-1`。

控制策略：

- 为三个分类集合分别写代表性测试
- 对规则外值显式定义为 `-1`，避免未定义行为

### 风险 3：中间语义和最终分类概念混淆

代码里同时存在 `semantic label` 和 `classification` 两套概念，若命名不清晰，后续维护容易误解。

控制策略：

- 仅把 `semantic label` 作为内部中间量使用
- 所有导出结构、PCD 字段、对外描述统一使用 `classification`

## 验收标准

- 导出的 PCD 不再包含字段 `semantic_label`
- 导出的 PCD 包含字段 `classification`
- `classification` 取值只出现 `-1`、`0`、`1`、`2`
- 现有灰度值到 `contiguous_id` 的映射逻辑保持不变
- 指定 `contiguous_id` 集合能正确归并到对应的 `classification`
- 未命中语义时仍输出 `classification = -1`
- 新增或更新对应单元测试并通过
