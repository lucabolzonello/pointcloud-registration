#ifndef POINT_CLOUD_HPP
#define POINT_CLOUD_HPP

#include "bounding_box.hpp"
#include "point.hpp"

#include <cstddef>
#include <vector>

namespace pcr::core {

/*
PointCloud. A class for representing a 3D PointCloud in point cloud registration
applications.
*/
class PointCloud {

public:
  // The data type of every coordinate in the PointCloud
  using coordinate_value_type = float;

  using point_type = Point<coordinate_value_type>;

  using point_iterator = std::vector<point_type>::iterator;

  using const_point_iterator = std::vector<point_type>::const_iterator;

  // Default Constructor
  // TimeComplexity:O(1)
  PointCloud();

  // PointCloud(Iterator), Construct from given iterator, leaves source intact
  // Time Complexity: O(1)
  template <typename Iterator>
  PointCloud(Iterator const begin, Iterator const end)
      : m_bounding_box(), data() {
    data.reserve(end - begin);
    for (Iterator point = begin; point != end; ++point) {
      this->add(*point);
    }
  }

  // size(), returns number of points in point cloud
  // Time Complexity: O(1)
  [[nodiscard]] std::size_t size() const noexcept;

  // reserve(), reserve memory for the n points to be stored in point cloud.
  // Slightly breaks abstraction but is crucial for efficiency.
  // Time Complexity: O(1)
  void reserve(std::size_t n);

  // is_empty(), returns true if there are no points in the point cloud
  // Time Complexity: O(1)
  [[nodiscard]] bool is_empty() const noexcept;

  // add(), add a point to the point cloud, and updates PointCloud metadata
  // Time Complexity: O(n) worst case, O(1) amortized
  void add(point_type p);

  // add(Iterator), add points from given iterator to the point cloud, leaves
  // source intact
  // Time Complexity: O(n) worst case, O(n) amortized
  template <typename Iterator>
  void add(Iterator const begin, Iterator const end) {
    for (Iterator point = begin; point != end; ++point) {
      this->add(*point);
    }
  }

  // begin(), returns iterator to beginning element
  // Time Complexity: O(1)
  [[nodiscard]] point_iterator begin();
  [[nodiscard]] const_point_iterator begin() const;
  [[nodiscard]] const_point_iterator cbegin() const;

  // end(), returns the past-the-end iterator
  // Time Complexity: O(1)
  [[nodiscard]] point_iterator end();
  [[nodiscard]] const_point_iterator end() const;
  [[nodiscard]] const_point_iterator cend() const;

  // get_bounding_box(), returns a copy of the BoundingBox for the PointCloud
  // Time Complexity: O(1)
  [[nodiscard]] BoundingBox<coordinate_value_type> get_bounding_box() const;

  // operator[](index), overload for the subscripting operator
  // Time Complexity: O(1)
  [[nodiscard]] inline point_type &operator[](size_t index) {
    return data[index];
  }

  // const operator[](index), const overload for the subscripting operator
  // Time Complexity: O(1)
  [[nodiscard]] inline const point_type &operator[](size_t index) const {
    return data[index];
  }

private:
  // Bounding box stats
  BoundingBox<coordinate_value_type> m_bounding_box;
  std::vector<point_type> data;
};

} // namespace pcr::core

#endif