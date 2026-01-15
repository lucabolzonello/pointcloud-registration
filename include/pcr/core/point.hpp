#ifndef POINT_HPP
#define POINT_HPP

namespace pcr::core {

/*
Point. A class template for representing a 3D Point in point cloud registration
applications.

The point class template has one parameter:
  - CoordType: The data type of the x, y, z coordinates.
 */
template <typename CoordType> class Point {

public:
  // x, y, z are public members because they will be accessed very frequently
  // and thus want to reduce friction to access
  CoordType x{};
  CoordType y{};
  CoordType z{};

  // Default Constructor
  constexpr Point() noexcept = default;

  // x, y, z parametrized constructor
  constexpr Point(CoordType x_, CoordType y_, CoordType z_) noexcept
      : x(x_), y(y_), z(z_) {}
};

// Operator== overload, compare points by value of x,y,z coordinates
template <typename PointType>
bool operator==(const PointType &lhs, const PointType &rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

}; // namespace pcr::core

#endif