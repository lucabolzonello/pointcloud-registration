
#include "pcr/spatial/kd_tree.hpp"

namespace pcr::spatial {
KdTree::KdTree() = default;

pcr::spatial::KdTree::KdTree(
    const std::vector<coordinate_value_type point_type::*> &dimensions)
    : m_dimensions(dimensions) {
  if (m_dimensions.empty() or m_dimensions.size() > 255) {
    throw std::runtime_error(
        "The KDTree must be indexed over  0 < K <= 255 dimensions");
  }
}

} // namespace pcr::spatial
