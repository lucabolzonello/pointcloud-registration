/**
 * @file kd_tree.hpp
 * @brief K-d tree spatial index for nearest neighbour queries
 */

#ifndef KD_TREE_HPP
#define KD_TREE_HPP

#include <cstddef>
#include <queue>
#include <type_traits>
#include <vector>

#include "pcr/core/bounding_box.hpp"
#include "pcr/core/point.hpp"
#include "pcr/core/point_cloud.hpp"
#include "pcr/prelude.hpp"

/**
 * @namespace pcr::spatial
 * @brief Core data structures for spatial storage and querying
 */
namespace pcr::spatial {

/**
 * @brief Item in the priority queue used in the KNN-Search
 *
 * Each node stores the squared_distance and the point_idx of the associated
 * point.
 */
struct PriorityQueueItem {
  /**
   * @brief Construct with the specified values
   *
   * @param point_cloud_idx Point cloud index of point
   * @param dist_squared squared distance between point and query point
   */
  PriorityQueueItem(pcr::point_idx point_cloud_idx, pcr::dist_t dist_squared)
      : m_point_cloud_idx(point_cloud_idx), m_dist_squared(dist_squared) {}

  pcr::point_idx m_point_cloud_idx; ///< Point cloud index of point (4 bytes)
  pcr::dist_t m_dist_squared; ///<  Squared distance between point and query
                              ///<  point (4 bytes)
};

/**
 * @brief K-d tree spatial index for fast nearest neighbor search
 *
 * Implements a k-d tree data structure for efficient spatial queries
 * on 3D point clouds. Supports k-nearest neighbor and radius search.
 * Tree is stored as a flat array for cache efficiency.
 *
 * @note The point cloud passed to build_index() must outlive this KdTree
 */
class KdTree {
public:
  /**
   * @brief Default constructor with to split along x, y, z planes
   *
   * Configures tree to split along x, y, and z planes.
   */
  KdTree();

  /**
   * @brief Construct with custom dimensions
   *
   * Allows splitting on arbitrary Point members.
   *
   * @param dimensions Vector of pointers-to-members specifying split dimensions
   */
  KdTree(const std::vector<pcr::coord_t pcr::point_t::*> &dimensions);

  /**
   * @brief Build spatial index from point cloud
   *
   * Constructs a balanced k-d tree by partitioning the point cloud.
   * Points in the point cloud will be reordered but not modified in value.
   *
   * Time Complexity: O(n log n) average case
   * Space Complexity: O(n) for tree nodes
   *
   * @param cloud Point cloud to index (will be reordered in-place)
   *
   * @warning cloud must outlive this KdTree - undefined behavior otherwise
   * @warning Points in the cloud will be reordered
   * @note Multiple build_index() calls will rebuild the tree
   */
  void build_index(core::PointCloud *cloud);

  /**
   * @brief Find k nearest neighbors to query point
   *
   * Searches the tree for the k points closest to query_point.
   * Results are unordered. If tree contains fewer than k points,
   * returns all available points.
   *
   * Time Complexity: O(log n + k) average case, O(n) worst case
   * Space Complexity: O(k)
   *
   * @param query_point Point to search from
   * @param k Number of nearest neighbors to find
   * @param[out] out_indices Vector to receive indices of nearest neighbors
   * @param[out] out_distances_squared Vector to receive squared distances
   *
   * @post out_indices.size() <= k
   * @post out_indices.size() == out_distances_squared.size()
   *
   * @warning build_index() must be called before using this function
   */
  void knn_search(const pcr::point_t &query_point, pcr::point_idx k,
                  std::vector<pcr::point_idx> &out_indices,
                  std::vector<pcr::dist_t> &out_distances_squared) const;

  /**
   * @brief Find all points within radius of query point
   *
   * Searches the tree for all points within the specified radius.
   * Results are unordered.
   *
   * Time Complexity: O(log n + m) average case where m is result count
   * Space Complexity: O(m)
   *
   * @param query_point Point to search from
   * @param radius Search radius
   * @param[out] out_indices Vector to receive indices of points within radius
   * @param[out] out_distances_squared Vector to receive squared distances
   *
   * @pre radius > 0
   * @post out_indices.size() == out_distances_squared.size()
   *
   * @warning build_index() must be called before using this function
   */
  void radius_search(const pcr::point_t &query_point, pcr::dist_t radius,
                     std::vector<pcr::point_idx> &out_indices,
                     std::vector<pcr::dist_t> &out_distances_squared) const;

private:
  /**
   * @brief Raw pointer to the input point cloud
   */
  pcr::core::PointCloud *m_point_cloud;

  /**
   * @brief Recursive helper for building tree
   *
   * Recursively partitions points and constructs balanced tree.
   *
   * @param left Index of leftmost element in current window
   * @param right Index one past rightmost element in current window
   * @param tree_idx Index in tree array where this node should be stored
   * @param split_plane Current dimension to split along
   */
  void build_index_rec(pcr::point_idx left, pcr::point_idx right,
                       pcr::point_idx tree_idx, uint8_t split_plane);

  /**
   * @brief Helper for getting the points coordinate value along the split
   * dimension
   *
   *
   *
   * @param query_point Reference to point
   * @param split_dimension Integer representing the split dimension
   */
  [[nodiscard]] static inline pcr::coord_t
  split_val(const pcr::point_t &query_point, uint8_t split_dimension) {
    switch (split_dimension) {
    case 0:
      return query_point.x;

    case 1:
      return query_point.y;

    default:
      return query_point.z;
    }
  }

  /**
   * @brief Get index of left child node
   *
   * @param curr_node_idx Index of current node
   * @return Index of left child in tree array
   */
  [[nodiscard]] static inline pcr::point_idx
  left_node(pcr::point_idx curr_node_idx) {
    return 2 * curr_node_idx + 1;
  }

  /**
   * @brief Get index of right child node
   *
   * @param curr_node_idx Index of current node
   * @return Index of right child in tree array
   */
  [[nodiscard]] static inline pcr::point_idx
  right_node(pcr::point_idx curr_node_idx) {
    return 2 * curr_node_idx + 2;
  }

  /**
   * @brief Get index of parent node
   *
   * @param curr_node_idx Index of current node
   * @return Index of parent in tree array
   */
  [[nodiscard]] static inline pcr::point_idx
  parent_node(pcr::point_idx curr_node_idx) {
    return (curr_node_idx - 1) / 2;
  }

  //  /**
  //   * @brief Checks if current node is a leaf node
  //   *
  //   * @param curr_node_idx Index of current node
  //   * @return Boolean indicating whether it is a leaf node
  //   */
  //  [[nodiscard]] static inline bool is_leaf_node(pcr::point_idx
  //  curr_node_idx) {
  //    // TO DO: Precompute (tree.size()/2 + 1) during build_index() to
  //    // avoid having to recompute each time
  //    return curr_idx >= (tree.size() / 2 + 1);
  //  }
  //
  /**
   * @brief Gets the current splitting dimension from the tree index of the given pivot point
   *
   * @param idx  Index of the pivot point to get the splitting dimension of
   * @return uint8_t representing the current splitting dimension 0 = x, 1 = y, 2 = z
   */
  [[nodiscard]] static inline uint8_t get_split_dim_from_idx(const pcr::point_idx &idx) noexcept {
    return pcr::log2(idx) + 1 % 3;
  }



  /**
   * @brief Gets the squared euclidean distance between two points
   *
   * @param p1 First point
   * @param p2 Second point
   * @return squared euclidean distance between two points
   */
  [[nodiscard]] static inline pcr::dist_t
  get_dist_squared(const pcr::point_t &p1, const pcr::point_t &p2) {
    pcr::dist_t x_diff = p1.x - p2.x;
    pcr::dist_t y_diff = p1.y - p2.y;
    pcr::dist_t z_diff = p1.z - p2.z;

    pcr::dist_t dist_squared =
        (x_diff * x_diff) + (y_diff * y_diff) + (z_diff * z_diff);
    return dist_squared;
  }

  /**
   * @brief Gets the squared euclidean distance between a point and an
   * axis-aligned bounding box
   *
   * This should be the shortest distance from the point to intersection with
   * bounding box
   *
   * @param query_point Query point
   * @param bounding_box Bounding Box
   * @return squared euclidean distance from the point to the
   */
  [[nodiscard]] static inline pcr::dist_t
  get_dist_squared(const pcr::point_t &query_point,
                   const core::BoundingBox<float> &bounding_box) {
    // Compute by coordinate clamping
    pcr::coord_t dx =
        query_point.x - std::max(bounding_box.min_x,
                                 std::min(query_point.x, bounding_box.max_x));
    pcr::coord_t dy =
        query_point.y - std::max(bounding_box.min_y,
                                 std::min(query_point.y, bounding_box.max_y));
    pcr::coord_t dz =
        query_point.z - std::max(bounding_box.min_z,
                                 std::min(query_point.z, bounding_box.max_z));

    return dx * dx + dy * dy + dz * dz;
  }

  /**
   * @brief Check if point is within radius of query point
   *
   * Fast distance test using squared Euclidean distance.
   *
   * @param p1 First point
   * @param p2 Second point
   * @param radius_squared Squared search radius
   * @return true if distance(p1, p2)^2 < radius_squared
   */
  [[nodiscard]] static inline bool in_radius(const pcr::point_t &p1,
                                             const pcr::point_t &p2,
                                             pcr::dist_t radius_squared) {

    return get_dist_squared(p1, p2) < radius_squared;
  }


  /**
   * @brief Recursive helper for knn_search
   *
   * @param query_point Point to search from
   * @param curr_idx The current index of the tree that we are at
   * @param k Number of nearest neighbors to find
   * @param bb The current bounding box of the pivot point we are at
   * @param[out] result_max_heap Priority queue that tracks the k-nearest-neighbours found so far.
   *
   * @post result_max_heap.size() == k
   */

  template <typename HeapType>
  void knn_search_rec(const pcr::point_t &query_point, pcr::point_idx curr_idx,
                      pcr::point_idx k, core::BoundingBox<pcr::coord_t> bb,
                      HeapType &result_max_heap) const;
};

} // namespace pcr::spatial

#endif // KD_TREE_HPP