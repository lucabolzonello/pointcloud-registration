/**
 * @file point.hpp
 * @brief 3D point representation for point cloud processing
 */

#ifndef POINT_HPP
#define POINT_HPP

/**
 * @namespace pcr::core
 * @brief Core data structures for point cloud registration
 */
namespace pcr::core {

/**
 * @brief Represents a 3D point in Euclidean space
 *
 * All coordinates are public members to minimize access overhead and API
 * simplicity.
 *
 * @tparam CoordType Type for coordinates (float or double)
 */
template <typename CoordType> class Point {
public:
  CoordType x{}; ///< X-coordinate
  CoordType y{}; ///< Y-coordinate
  CoordType z{}; ///< Z-coordinate

  /**
   * @brief Default constructor - initializes point to origin (0, 0, 0)
   */
  constexpr Point() noexcept = default;

  /**
   * @brief Construct point with specified coordinates
   *
   * @param x_ X-coordinate value
   * @param y_ Y-coordinate value
   * @param z_ Z-coordinate value
   */
  constexpr Point(CoordType x_, CoordType y_, CoordType z_) noexcept
      : x(x_), y(y_), z(z_) {}
};

} // namespace pcr::core

#endif // POINT_HPP