#ifndef POINT_CLOUD_HPP
#define POINT_CLOUD_HPP

#include "pcr/prelude.hpp"
#include "point.hpp"
#include <vector>

/**
 * @namespace pcr::core
 * @brief Core data structures for point cloud registration
 */
namespace pcr::core {

/**
 * @brief Container for 3D point cloud data
 *
 * Manages and owns a collection of 3D points
 */
using PointCloud = std::vector<pcr::point_t>;
} // namespace pcr::core

#endif // POINT_CLOUD_HPP