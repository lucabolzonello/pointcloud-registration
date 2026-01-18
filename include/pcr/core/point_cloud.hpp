#ifndef POINT_CLOUD_HPP
#define POINT_CLOUD_HPP

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
  PointCloud();

  // PointCloud(Iterator), Construct from given iterator, leaves source intact
  template <typename Iterator>
  PointCloud(Iterator const first, Iterator const last)
      : m_points(first, last) {}

  // size(), returns number of points in point cloud
  [[nodiscard]] std::size_t size() const noexcept;

  // reserve(), reserve memory for the n points to be stored in point cloud.
  // Slightly breaks abstraction but is crucial for efficiency.
  void reserve(std::size_t n);

  // is_empty(), returns true if there are no points in the point cloud
  [[nodiscard]] bool is_empty() const noexcept;

  // add(), add a point to the point cloud
  void add(point_type p);

  // add(Iterator), add points from given iterator to the point cloud, leaves
  // source intact
  template <typename Iterator>
  void add(Iterator const first, Iterator const last) {
    m_points.insert(m_points.end(), first, last);
  }

  // begin(), returns iterator to beginning element
  [[nodiscard]] point_iterator begin();
  [[nodiscard]] const_point_iterator cbegin() const;

  // end(), returns the past-the-end iterator
  [[nodiscard]] point_iterator end();
  [[nodiscard]] const_point_iterator cend() const;

private:
  std::vector<point_type> m_points;
};

} // namespace pcr::core

#endif