#ifndef KD_TREE_HPP
#define KD_TREE_HPP

#include <cstddef>
#include <type_traits>
#include <vector>

#include "pcr/core/point.hpp"
#include "pcr/core/point_cloud.hpp"

namespace pcr::spatial {

class KdTree {
public:
  using point_type = core::PointCloud::point_type;
  using coordinate_value_type = core::PointCloud::coordinate_value_type;

  // KdTree() , automatically uses Point.x, Point.y,
  // Point.z as the Pointer to Member dimensions for indexing
  KdTree();

  // KdTree(dimensions), takes a vector of Pointer to Member's
  // within the Point class as the dimensions for indexing
  KdTree(
      const std::vector<core::PointCloud::coordinate_value_type point_type::*>
          dimensions);

  // build_index(cloud)
  // Note: It is assumed that the cloud must outlive this KdTree
  // object, undefined behaviour if caller doesn't follow this guideline
  void build_index(const core::PointCloud &cloud);

  // knn_search(query_point, k, &out_indices, &out_distances_squared), Get the K
  // nearest neighbours to the query_point
  // Note: if there is less than k points in
  // the tree Note: out_indices may contain less than k points
  void
  knn_search(const point_type &query_point, size_t k,
             std::vector<size_t> &out_indices,
             std::vector<coordinate_value_type> &out_distances_squared) const;

  // radius_search(), Query the spatial index to get all points within the
  // radius "radius" of query_point
  void radius_search(
      const core::PointCloud::point_type &query_point,
      coordinate_value_type radius, std::vector<size_t> &out_indices,
      std::vector<coordinate_value_type> &out_distances_squared) const;
};

} // namespace pcr::spatial

#endif
