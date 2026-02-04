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
      : min_x(std::numeric_limits<T>::max()),
        min_y(std::numeric_limits<T>::max()),
        min_z(std::numeric_limits<T>::max()),
        max_x(std::numeric_limits<T>::lowest()),
        max_y(std::numeric_limits<T>::lowest()),
        max_z(std::numeric_limits<T>::lowest()) {}

  T min_x; ///< Minimum X-coordinate of bounding box
  T min_y; ///< Minimum Y-coordinate of bounding box
  T min_z; ///< Minimum Z-coordinate of bounding box
  T max_x; ///< Maximum X-coordinate of bounding box
  T max_y; ///< Maximum Y-coordinate of bounding box
  T max_z; ///< Maximum Z-coordinate of bounding box
};

} // namespace pcr::core

#endif // BOUNDING_BOX_HPP