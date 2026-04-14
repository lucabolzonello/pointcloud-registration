/**
 * @file point.hpp
 * @brief 3D point representation for point cloud processing
 */

#ifndef POINT_HPP
#define POINT_HPP

#include <Eigen/Dense>

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
  /**
   * @brief Default constructor - initializes point to origin (0, 0, 0)
   */
  constexpr Point() noexcept : coords(Eigen::Vector3<CoordType>::Zero()) {
  }


  /**
   * @brief Construct point with specified coordinates
   *
   * @param x_ X-coordinate value
   * @param y_ Y-coordinate value
   * @param z_ Z-coordinate value
   */
  constexpr Point(CoordType x_, CoordType y_, CoordType z_) noexcept
    : coords(Eigen::Vector3<CoordType>(x_, y_, z_)) {
  }


  /**
   * @brief Getter for the x coordinate
   * @return Copy of x coordinate
   */
  CoordType x() const noexcept { return coords[0]; }

  /**
  * @brief Getter for the x coordinate
  * @return Reference to x coordinate
  */
  CoordType &x() noexcept { return coords[0]; }

  /**
  * @brief Getter for the y coordinate
  * @return Copy of y coordinate
  */
  CoordType y() const noexcept { return coords[1]; }
  /**
   * @brief Getter for the y coordinate
   * @return Reference to y coordinate
   */
  CoordType &y() noexcept { return coords[1]; }

  /**
  * @brief Getter for the z coordinate
  * @return Copy of z coordinate
  */
  CoordType z() const noexcept { return coords[2]; }


  /**
  * @brief Getter for the z coordinate
  * @return Reference to z coordinate
  */
  CoordType &z() noexcept { return coords[2]; }


  /**
   * @brief Operator overload for addition assignment with another point
   * @param p point to add to the current point
   * @return Current point after addition
   */
  Point &operator+=(const Point &p) noexcept {
    coords += p.coords;
    return *this;
  }

  /**
   * @brief Operator overload for division assignment
   * @param s scalar to divide the current point by
   * @return Current point after division
   */
  Point &operator/=(const CoordType &s) noexcept {
    coords /= s;
    return *this;
  }


  /**
   * @param p  Copy of point to divide
   * @param s  Scalar to divide by
   * @return New point after division
   */
  template <typename T>
  friend Point operator/(Point p, const T &s) noexcept {
    p /= s;
    return p;
  }

  /**
   *
   * @param lhs Left point to add
   * @param rhs Right point to add
   * @return New point after addition
   */
  friend Point operator+(Point lhs, const Point &rhs) noexcept {
    lhs += rhs;
    return lhs;
  }

private:
  Eigen::Matrix<CoordType, 3, 1> coords;
  ///< Homogeneous Coordinates as Eigen vector


};


} // namespace pcr::core


#endif // POINT_HPP