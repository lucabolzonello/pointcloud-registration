/**
 * @file kd_tree.hpp
 * @brief K-d tree spatial index for nearest neighbour queries
 */

#ifndef KD_TREE_HPP
#define KD_TREE_HPP

#include <cstddef>
#include <type_traits>
#include <vector>

#include "pcr/core/point.hpp"
#include "pcr/core/point_cloud.hpp"

/**
 * @namespace pcr::spatial
 * @brief Core data structures for spatial storage and querying
 */
namespace pcr::spatial {

/**
 * @brief Node in the k-d tree structure
 *
 * Compact kd-tree node
 * Each node stores the splitting plane and float value.
 */
struct KdTreeNode {
  float split_value;   ///< Coordinate value at which to split (4 bytes)
  uint8_t split_plane; ///< Dimension index to split on (1 byte)
}; // 8 byte aligned

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
  using point_type = core::PointCloud::point_type; ///< Point type
  using coordinate_value_type =
      core::PointCloud::coordinate_value_type; ///< Coordinate type

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
  KdTree(const std::vector<coordinate_value_type point_type::*> &dimensions);

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
  void build_index(core::PointCloud &cloud);

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
  void
  knn_search(const point_type &query_point, size_t k,
             std::vector<size_t> &out_indices,
             std::vector<coordinate_value_type> &out_distances_squared) const;

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
  void radius_search(
      const core::PointCloud::point_type &query_point,
      coordinate_value_type radius, std::vector<size_t> &out_indices,
      std::vector<coordinate_value_type> &out_distances_squared) const;

private:
  /**
   * @brief Dimensions to use for splitting (defaults to x, y, z)
   */
  const std::vector<coordinate_value_type point_type::*> m_dimensions = {
      &point_type::x, &point_type::y, &point_type::z};

  /**
   * @brief Tree stored as flat array
   */
  std::vector<KdTreeNode> tree;

  /**
   * @brief Recursive helper for building tree
   *
   * Recursively partitions points and constructs balanced tree.
   *
   * @param cloud Reference to point cloud being indexed
   * @param left Index of leftmost element in current window
   * @param right Index one past rightmost element in current window
   * @param tree_idx Index in tree array where this node should be stored
   * @param split_plane Current dimension to split along
   */
  void build_index_rec(pcr::core::PointCloud &cloud, long left, long right,
                       long tree_idx, uint8_t split_plane);
};

} // namespace pcr::spatial

#endif // KD_TREE_HPP