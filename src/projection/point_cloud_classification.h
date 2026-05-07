#pragma once

namespace segment_projection::projection {

// 负值和未映射的 contiguous_id 统一返回 -1。
int MapContiguousIdToClassification(int contiguous_id);

}  // namespace segment_projection::projection
