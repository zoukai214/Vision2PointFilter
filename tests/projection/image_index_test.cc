#include "projection/image_index.h"

#include <filesystem>
#include <iostream>

int main() {
  const std::filesystem::path image_dir =
      "/workspace/GACRT026_1758521322/images_seg_mask2former/front_wide";

  const auto index =
      segment_projection::projection::ImageIndex::Build(image_dir);
  if (!index) {
    std::cerr << "ImageIndex::Build returned null\n";
    return 1;
  }

  const auto match = index->FindNearestOrNull(1758521322500LL);
  if (!match) {
    std::cerr << "FindNearestOrNull returned null\n";
    return 1;
  }
  if (match->timestamp_ms != 1758521322499LL) {
    std::cerr << "unexpected matched timestamp: " << match->timestamp_ms
              << "\n";
    return 1;
  }
  if (match->delta_ms != 1LL) {
    std::cerr << "unexpected match delta: " << match->delta_ms << "\n";
    return 1;
  }
  if (match->path.filename() != "1758521322499000000_50_0.png") {
    std::cerr << "unexpected matched path: " << match->path << "\n";
    return 1;
  }

  return 0;
}
