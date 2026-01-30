#ifndef BOUNDING_BOX_HPP
#define BOUNDING_BOX_HPP
#include <limits>
namespace pcr::core {
// Struct to represent a 3-Dimensional bounding box
template <typename T> struct BoundingBox {
  BoundingBox()
      : min_x(std::numeric_limits<T>::max()),
        min_y(std::numeric_limits<T>::max()),
        min_z(std::numeric_limits<T>::max()),
        max_x(std::numeric_limits<T>::lowest()),
        max_y(std::numeric_limits<T>::lowest()),
        max_z(std::numeric_limits<T>::lowest()) {}

  T min_x;
  T min_y;
  T min_z;
  T max_x;
  T max_y;
  T max_z;
};

} // namespace pcr::core
#endif