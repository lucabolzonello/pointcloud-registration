#include "pcr/spatial/kd_tree.hpp"
#include <algorithm>
#include <iostream>

namespace pcr::spatial {

KdTree::KdTree() : m_point_cloud(nullptr) {
}

void KdTree::build_index(pcr::core::PointCloud *cloud) {
  // Set m_point_cloud to point to the point_cloud
  m_point_cloud = cloud;
  m_point_cloud_size = cloud->size();

  // Recursively build index
  build_index_rec(0, m_point_cloud_size, 0);
}

void KdTree::build_index_rec(pcr::point_idx left, pcr::point_idx right,
                             uint8_t split_plane) {

  pcr::point_idx num_elements = right - left;

  // Stop recursion if leaf node
  if (num_elements <= 0) {
    return;
  }
  if (num_elements == 1) {
    return;
  }

  // determine midpoint between right and left
  pcr::point_idx midpoint = left + num_elements / 2;

  // call nth_element to find the median and place it at midpoint_it
  std::nth_element(
      m_point_cloud->begin() + left, m_point_cloud->begin() + midpoint,
      m_point_cloud->begin() + right,
      [&split_plane, this](const auto &lhs, const auto &rhs) {
        return split_val(lhs, split_plane) < split_val(rhs, split_plane);
      });

  // increment split_plane
  split_plane = (split_plane + 1) % 3;

  // Recurse left side
  build_index_rec(left, midpoint, split_plane);

  // Recurse right side
  build_index_rec(midpoint + 1, right, split_plane);
}

void KdTree::knn_search(const pcr::point_t &query_point, pcr::point_idx k,
                        std::vector<pcr::point_idx> &out_indices,
                        std::vector<pcr::dist_t> &out_distances_squared) const {

  // if k > m_point_cloud.size(), return all points
  if (m_point_cloud->size() <= k) {
    out_indices.reserve(m_point_cloud->size());
    out_distances_squared.reserve(m_point_cloud->size());

    for (size_t i = 0; i < m_point_cloud->size(); ++i) {
      out_indices.push_back(i);
      out_distances_squared.push_back(
          get_dist_squared(query_point, (*m_point_cloud)[i]));
    }
    return;
  }

  // Define comparison lambda for the priority queue
  auto cmp = [](PriorityQueueItem &left, PriorityQueueItem &right) {
    return left.m_dist_squared < right.m_dist_squared;
  };

  // Pre-reserve priority queue vector to prevent multiple reallocations while
  // using the priority queue
  std::vector<PriorityQueueItem> priority_queue_vector;
  priority_queue_vector.reserve(k);
  std::priority_queue<PriorityQueueItem, std::vector<PriorityQueueItem>,
                      decltype(cmp)>
      result_heap(cmp, std::move(priority_queue_vector));

  // perform recursive search
  knn_search_rec(query_point, k, 0, m_point_cloud_size,
                 0, pcr::core::BoundingBox<pcr::coord_t>(),
                 result_heap);

  // Grab from the result heap and place in out_indices_and_distances_squared
  out_indices.reserve(k);
  out_distances_squared.reserve(k);
  while (!result_heap.empty()) {
    auto top = result_heap.top();
    result_heap.pop();
    out_indices.emplace_back(top.m_point_cloud_idx);
    out_distances_squared.emplace_back(top.m_dist_squared);
  }
}

template <typename HeapType>
void KdTree::knn_search_rec(const pcr::point_t &query_point, pcr::point_idx k,
                            pcr::point_idx left, pcr::point_idx right,
                            uint8_t split_dim,
                            const pcr::core::BoundingBox<pcr::coord_t> &bb,
                            HeapType &result_max_heap) const {
  // If past leaf node then return
  if (left == right)
    return;

  pcr::point_idx curr_idx = left + (right - left) / 2;

  // Check current node, add to heap if it one of the K-nearest-neighbours
  // found so far
  pcr::dist_t dist_to_point =
      get_dist_squared((*m_point_cloud)[curr_idx], query_point);

  if (result_max_heap.size() < k) {
    // push onto the heap
    result_max_heap.emplace(curr_idx, dist_to_point);

  } else if (dist_to_point < result_max_heap.top().m_dist_squared) {
    // pop current max_element and push onto heap
    result_max_heap.pop();
    result_max_heap.emplace(curr_idx, dist_to_point);
  }

  // Determine which side of the split plane the query point is on
  coord_t split_value =
      split_val((*m_point_cloud)[curr_idx], split_dim);

  pcr::point_idx near_left, near_right;
  pcr::point_idx far_left, far_right;
  pcr::core::BoundingBox<pcr::coord_t> nearer_bb = bb;
  pcr::core::BoundingBox<pcr::coord_t> farther_bb = bb;

  if (split_val(query_point, split_dim) < split_value) {
    near_left = left;
    near_right = curr_idx;
    far_left = curr_idx + 1;
    far_right = right;
    nearer_bb.split(true, split_dim, split_value);
    farther_bb.split(false, split_dim, split_value);

  } else {
    near_left = curr_idx + 1;
    near_right = right;
    far_left = left;
    far_right = curr_idx;
    nearer_bb.split(false, split_dim, split_value);
    farther_bb.split(true, split_dim, split_value);
  }
  split_dim = (split_dim + 1) % 3;

  // Recurse near-side
  knn_search_rec(query_point, k, near_left, near_right, split_dim, nearer_bb,
                 result_max_heap);

  // Prune farther_bb path if the bounding box's distance is further than max dist
  if (get_dist_squared(query_point, farther_bb, split_dim) <
      result_max_heap
      .top().m_dist_squared) {
    // Recurse far side
    knn_search_rec(query_point, k, far_left, far_right, split_dim, farther_bb,
                   result_max_heap);
  }

}

void KdTree::radius_search(
    const pcr::point_t &query_point, pcr::dist_t radius,
    std::vector<pcr::point_idx> &out_indices,
    std::vector<pcr::dist_t> &out_distances_squared) const {
  // perform recursive search

  radius_search_rec(query_point, radius * radius, 0, m_point_cloud_size, 0,
                    pcr::core::BoundingBox<pcr::coord_t>(),
                    out_indices, out_distances_squared);

}

void KdTree::radius_search_rec(const pcr::point_t &query_point,
                               pcr::dist_t radius_squared,
                               pcr::point_idx left, pcr::point_idx right,
                               uint8_t split_dim,
                               const pcr::core::BoundingBox<pcr::coord_t> &bb,
                               std::vector<pcr::point_idx> &out_indices,
                               std::vector<pcr::dist_t> &
                               out_distances_squared)
const {

  // If past leaf node then return
  if (left == right)
    return;

  pcr::point_idx curr_idx = left + (right - left) / 2;

  // Check current node, add to heap if it one of the K-nearest-neighbours
  // found so far
  pcr::dist_t dist_to_point =
      get_dist_squared((*m_point_cloud)[curr_idx], query_point);

  if (dist_to_point < radius_squared) {
    out_indices.push_back(curr_idx);
    out_distances_squared.push_back(dist_to_point);
  }

  // Determine which side of the split plane the query point is on
  coord_t split_value =
      split_val((*m_point_cloud)[curr_idx], split_dim);

  pcr::point_idx near_left, near_right;
  pcr::point_idx far_left, far_right;
  pcr::core::BoundingBox<pcr::coord_t> nearer_bb = bb;
  pcr::core::BoundingBox<pcr::coord_t> farther_bb = bb;

  if (split_val(query_point, split_dim) < split_value) {
    near_left = left;
    near_right = curr_idx;
    far_left = curr_idx + 1;
    far_right = right;
    nearer_bb.split(true, split_dim, split_value);
    farther_bb.split(false, split_dim, split_value);

  } else {
    near_left = curr_idx + 1;
    near_right = right;
    far_left = left;
    far_right = curr_idx;
    nearer_bb.split(false, split_dim, split_value);
    farther_bb.split(true, split_dim, split_value);
  }

  split_dim = (split_dim + 1) % 3;

  // Recurse near-side
  radius_search_rec(query_point, radius_squared, near_left, near_right,
                    split_dim,
                    nearer_bb, out_indices, out_distances_squared);

  // Prune farther_bb path if the bounding box's distance is further than max dist
  if (get_dist_squared(query_point, farther_bb, split_dim) < radius_squared) {
    // Recurse far side
    radius_search_rec(query_point, radius_squared, far_left, far_right,
                      split_dim,
                      farther_bb,
                      out_indices, out_distances_squared);
  }
}

} // namespace pcr::spatial
