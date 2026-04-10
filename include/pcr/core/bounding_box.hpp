/**
 * @file bounding_box.hpp
 * @brief Axis-aligned bounding box for spatial queries
 */

#ifndef BOUNDING_BOX_HPP
#define BOUNDING_BOX_HPP

#include <limits>

/**
 * @namespace pcr::core
 * @brief Core data structures for point cloud registration
 */
namespace pcr::core {

/**
 * @brief 3D Spatial Bounding Box
 *
 * Represents 3D bounding of a point cloud. Initialized with inverted bounds to
 * simplify incremental updates.
 *
 * @tparam T Type for coordinates (float or double)
 */
template <typename T> struct BoundingBox {
  /**
   * @brief Default constructor - initializes inverted bounds
   *
   * Min values are set to numeric max, max values to numeric lowest when
   * initialized.
   */
  BoundingBox()
    : min_x(std::numeric_limits<T>::lowest()),
      min_y(std::numeric_limits<T>::lowest()),
      min_z(std::numeric_limits<T>::lowest()),
      max_x(std::numeric_limits<T>::max()),
      max_y(std::numeric_limits<T>::max()),
      max_z(std::numeric_limits<T>::max()) {
  }

  /**
   * @brief Copy Constructor
   *
   * Copies the bounding box.
   *
   * @param other The bounding box to copy
   */
  BoundingBox(const BoundingBox &other) = default;

  /**
   * @brief Split the bounding box along a dimension
   *
   * Modifies the bounding box to contain either the left or right side of the
   * split. If left_side is true, the bounding box will contain the left side of
   * the split. Otherwise, the bounding box will contain the right side of the
   * split.
   *
   * @param left_side Whether to take the left side of the split or the right
   * side
   * @param split_dimension The dimension to split along (0 for x, 1 for y, 2
   * for z)
   */
  void split(bool left_side, uint8_t split_dimension, T split_val) {
    if (split_dimension == 0) {
      if (left_side) {
        max_x = split_val;
      } else {
        min_x = split_val;
      }
    } else if (split_dimension == 1) {
      if (left_side) {
        max_y = split_val;
      } else {
        min_y = split_val;
      }
    } else if (split_dimension == 2) {
      if (left_side) {
        max_z = split_val;
      } else {
        min_z = split_val;
      }
    }
  }

  T min_x; ///< Minimum X-coordinate of bounding box
  T min_y; ///< Minimum Y-coordinate of bounding box
  T min_z; ///< Minimum Z-coordinate of bounding box
  T max_x; ///< Maximum X-coordinate of bounding box
  T max_y; ///< Maximum Y-coordinate of bounding box
  T max_z; ///< Maximum Z-coordinate of bounding box
};

} // namespace pcr::core

#endif // BOUNDING_BOX_HPP