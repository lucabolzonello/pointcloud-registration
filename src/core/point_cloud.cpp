#include "pcr/core/point_cloud.hpp"
#include <vector>

namespace pcr::core {
// Default Constructor
PointCloud::PointCloud() : m_points() {}

// size(), returns number of points in point cloud
std::size_t PointCloud::size() const noexcept { return m_points.size(); }

// reserve(), reserve memory for the n points to be stored in point cloud.
// Slightly breaks abstraction but is crucial for efficiency.
void PointCloud::reserve(std::size_t n) { m_points.reserve(n); }

// is_empty(), returns true if there are no points in the point cloud
bool PointCloud::is_empty() const noexcept { return this->size() == 0; }

// add(), add a point to the point cloud
void PointCloud::add(point_type p) { m_points.push_back(p); }

// begin(), returns iterator to beginning element
PointCloud::point_iterator PointCloud::begin() { return m_points.begin(); }
PointCloud::const_point_iterator PointCloud::begin() const {
  return m_points.begin();
}
PointCloud::const_point_iterator PointCloud::cbegin() const {
  return m_points.cbegin();
}

// end(), returns the past-the-end iterator
PointCloud::point_iterator PointCloud::end() { return m_points.end(); }
PointCloud::const_point_iterator PointCloud::end() const {
  return m_points.end();
}
PointCloud::const_point_iterator PointCloud::cend() const {
  return m_points.cend();
}
} // namespace pcr::core