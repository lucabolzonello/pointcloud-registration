#ifndef BOUNDING_BOX_HPP
#define BOUNDING_BOX_HPP
namespace pcr::core {
// Struct to represent a 3-Dimensional bounding box
template <typename T> struct BoundingBox {
  T min_x;
  T min_y;
  T min_z;
  T max_x;
  T max_y;
  T max_z;
};

} // namespace pcr::core
#endif