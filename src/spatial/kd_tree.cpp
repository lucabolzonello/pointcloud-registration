#include <algorithm>

#include "pcr/spatial/kd_tree.hpp"

namespace pcr::spatial {
KdTree::KdTree() = default;

pcr::spatial::KdTree::KdTree(
    const std::vector<pcr::coord_t pcr::point_t::*> &dimensions)
    : m_dimensions(dimensions) {
  if (m_dimensions.empty() or m_dimensions.size() > 255) {
    throw std::runtime_error(
        "The KDTree must be indexed over  0 < K <= 255 dimensions");
  }
}

void KdTree::build_index(pcr::core::PointCloud &cloud) {
  // reserve capacity in tree ahead of time to prevent repeated resizes
  tree.reserve(cloud.size());

  // Recursively build index
  build_index_rec(cloud, 0, cloud.size(), 0, 0);
}

void KdTree::build_index_rec(pcr::core::PointCloud &cloud, pcr::point_idx left,
                             pcr::point_idx right, pcr::point_idx tree_idx,
                             uint8_t split_plane) {

  pcr::point_idx num_elements = right - left;

  // Stop recursion if leaf node
  if (num_elements <= 0) {
    return;
  }
  if (num_elements == 1) {
    tree[tree_idx] = {cloud[left].*(m_dimensions[split_plane]), split_plane};
    return;
  }

  // determine midpoint between right and left
  pcr::point_idx midpoint = left + num_elements / 2;

  // call nth_element to find the median and place it at midpoint_it
  std::nth_element(cloud.begin() + left, cloud.begin() + midpoint,
                   cloud.begin() + right,
                   [&split_plane, this](const auto &lhs, const auto &rhs) {
                     return lhs.*(m_dimensions[split_plane]) <
                            rhs.*(m_dimensions[split_plane]);
                   });

  // Add midpoint to tree, this is the split node
  tree[tree_idx] = {cloud[midpoint].*(m_dimensions[split_plane]), split_plane};

  // increment split_plane
  split_plane = (split_plane + 1) % m_dimensions.size();

  // Recurse left side
  build_index_rec(cloud, left, midpoint, 2 * tree_idx + 1, split_plane);

  // Recurse right side
  build_index_rec(cloud, midpoint + 1, right, 2 * tree_idx + 2, split_plane);
}

void KdTree::knn_search(const pcr::point_t &query_point, pcr::point_idx k,
                        std::vector<pcr::point_idx> &out_indices,
                        std::vector<pcr::dist_t> &out_distances_squared) const {
}

void KdTree::radius_search(
    const pcr::point_t &query_point, pcr::dist_t radius,
    std::vector<pcr::point_idx> &out_indices,
    std::vector<pcr::dist_t> &out_distances_squared) const {}
} // namespace pcr::spatial
