#include "projection/image_index.h"

#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
  const std::filesystem::path temp_dir =
      std::filesystem::temp_directory_path() /
      std::filesystem::path("image_index_test_fixture");
  std::error_code ec;
  std::filesystem::remove_all(temp_dir, ec);
  std::filesystem::create_directories(temp_dir, ec);
  if (ec) {
    std::cerr << "failed to create temp dir: " << ec.message() << "\n";
    return 1;
  }

  {
    std::ofstream(temp_dir / "1758521322499000000_50_0.png").put('\n');
    std::ofstream(temp_dir / "1758521322598000000_50_1.png").put('\n');
    std::ofstream(temp_dir / "1758521322698000000_50_2.png").put('\n');
    std::ofstream(temp_dir / "ignore.txt").put('\n');
    std::ofstream(temp_dir / "not_a_timestamp.png").put('\n');
  }

  const auto index = segment_projection::projection::ImageIndex::Build(temp_dir);
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
  const auto file_instead_of_dir = temp_dir / "existing_file";
  std::ofstream(file_instead_of_dir) << "x";
  if (segment_projection::projection::ImageIndex::Build(file_instead_of_dir)) {
    std::cerr << "expected Build to fail for non-directory path\n";
    return 1;
  }

  return 0;
}
