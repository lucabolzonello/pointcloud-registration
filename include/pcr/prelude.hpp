/**
 * @file prelude.hpp
 * @brief for
 */
#ifndef PRELUDE_HPP
#define PRELUDE_HPP
#include <cstdint>
#include <type_traits>

#include "pcr/core/point.hpp"

/**
 * @namespace pcr
 * @brief Core definitions to be used across project
 */
namespace pcr {

using coord_t = float;  ///< Data type for all coordinates
using dist_t = coord_t; /// < Data type for distances

using point_idx = uint32_t; ///< Data type for indexing into PointCloud

using point_t = core::Point<coord_t>; ///< Data type for a 3D point

// Coordinate type must be floating point (float, double, long double)
static_assert(std::is_floating_point_v<coord_t>);

// Distance type must be floating point (float, double, long double)
static_assert(std::is_floating_point_v<dist_t>);

} // namespace pcr

#endif // PRELUDE_HPP