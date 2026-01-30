#include "pcr/core/point_cloud.hpp"
#include <vector>

namespace pcr::core {
// Default Constructor
PointCloud::PointCloud() : m_bounding_box(), data() {}

// size(), returns number of points in point cloud
std::size_t PointCloud::size() const noexcept { return data.size(); }

// reserve(), reserve memory for the n points to be stored in point cloud.
// Slightly breaks abstraction but is crucial for efficiency.
void PointCloud::reserve(std::size_t n) { data.reserve(n); }

// is_empty(), returns true if there are no points in the point cloud
bool PointCloud::is_empty() const noexcept { return this->size() == 0; }

// add(), add a point to the point cloud
void PointCloud::add(point_type p) {
  // update bounding_box
  if (p.x < m_bounding_box.min_x)
    m_bounding_box.min_x = p.x;
  else if (p.x > m_bounding_box.max_x)
    m_bounding_box.max_x = p.x;

  if (p.x < m_bounding_box.min_x)
    m_bounding_box.min_x = p.x;
  if (p.x > m_bounding_box.max_x)
    m_bounding_box.max_x = p.x;

  if (p.y < m_bounding_box.min_y)
    m_bounding_box.min_y = p.y;
  if (p.y > m_bounding_box.max_y)
    m_bounding_box.max_y = p.y;

  if (p.z < m_bounding_box.min_z)
    m_bounding_box.min_z = p.z;
  if (p.z > m_bounding_box.max_z)
    m_bounding_box.max_z = p.z;

  data.push_back(p);
}

// begin(),
PointCloud::point_iterator PointCloud::begin() { return data.begin(); }
PointCloud::const_point_iterator PointCloud::begin() const {
  return data.begin();
}
PointCloud::const_point_iterator PointCloud::cbegin() const {
  return data.cbegin();
}

// end(), returns the past-the-end iterator
PointCloud::point_iterator PointCloud::end() { return data.end(); }
PointCloud::const_point_iterator PointCloud::end() const { return data.end(); }
PointCloud::const_point_iterator PointCloud::cend() const {
  return data.cend();
}
// get_bounding_box(), returns a copy of the BoundingBox for the PointCloud
// Time Complexity: O(1)
[[nodiscard]] BoundingBox<PointCloud::coordinate_value_type>
PointCloud::get_bounding_box() const {
  return m_bounding_box;
}


} // namespace pcr::core