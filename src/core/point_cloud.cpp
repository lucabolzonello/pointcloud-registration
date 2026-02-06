#include "pcr/core/point_cloud.hpp"
#include <vector>

namespace pcr::core {
PointCloud::PointCloud() : m_bounding_box(), data() {}

std::size_t PointCloud::size() const noexcept { return data.size(); }

void PointCloud::reserve(std::size_t n) { data.reserve(n); }

bool PointCloud::is_empty() const noexcept { return this->size() == 0; }

void PointCloud::add(pcr::point_t p) {
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

  // add point
  data.push_back(p);
}

PointCloud::point_iterator PointCloud::begin() { return data.begin(); }

PointCloud::const_point_iterator PointCloud::begin() const {
  return data.begin();
}

PointCloud::const_point_iterator PointCloud::cbegin() const {
  return data.cbegin();
}

PointCloud::point_iterator PointCloud::end() { return data.end(); }

PointCloud::const_point_iterator PointCloud::end() const { return data.end(); }

PointCloud::const_point_iterator PointCloud::cend() const {
  return data.cend();
}

[[nodiscard]] BoundingBox<pcr::coord_t> PointCloud::get_bounding_box() const {
  return m_bounding_box;
}

} // namespace pcr::core