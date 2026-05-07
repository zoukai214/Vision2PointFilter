#include "data_loader/semantic_labeled_point.h"

#include <iostream>
#include <type_traits>
#include <vector>

#include <pcl/common/io.h>

namespace {

template <typename T, typename = void>
struct HasClassificationMember : std::false_type {};

template <typename T>
struct HasClassificationMember<
    T, std::void_t<decltype(std::declval<T&>().classification)>>
    : std::true_type {};

template <typename T, typename = void>
struct HasSemanticLabelMember : std::false_type {};

template <typename T>
struct HasSemanticLabelMember<
    T, std::void_t<decltype(std::declval<T&>().semantic_label)>>
    : std::true_type {};

}  // namespace

static_assert(
    HasClassificationMember<segment_projection::data_loader::SemanticLabeledPoint>::value,
    "SemanticLabeledPoint should expose a classification member");
static_assert(
    !HasSemanticLabelMember<segment_projection::data_loader::SemanticLabeledPoint>::value,
    "SemanticLabeledPoint should not expose a semantic_label member");

int main() {
  const std::vector<pcl::PCLPointField> fields =
      pcl::getFields<segment_projection::data_loader::SemanticLabeledPoint>();

  bool has_classification = false;
  bool has_semantic_label = false;
  for (const auto& field : fields) {
    if (field.name == "classification") {
      has_classification = true;
    }
    if (field.name == "semantic_label") {
      has_semantic_label = true;
    }
  }

  if (!has_classification) {
    std::cerr << "expected classification field to be registered\n";
    return 1;
  }

  if (has_semantic_label) {
    std::cerr << "semantic_label field should no longer be registered\n";
    return 1;
  }

  return 0;
}
