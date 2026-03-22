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

using coord_t = float; ///< Data type for all coordinates
using dist_t = coord_t; /// < Data type for distances

// CAUTION: If you change point_idx, you must also change log2 bit width
using point_idx = uint32_t; ///< Data type for indexing into PointCloud

/**
 * @brief Computes the base 2 logarithm of the point_idx unsigned integer
 * @note Must be updated to match the bitwidth of point_idx
 * @param n
 * @return The logarithmic base for the given point_idx unsigned integer
 */
[[nodiscard]] inline uint8_t log2(point_idx n) noexcept {
#if defined(_MSC_VER)
  unsigned long index;
  _BitScanReverse(&index, n);
  return static_cast<uint8_t>(index);
#elif defined(__GNUC__) || defined(__clang__)
  return static_cast<uint8_t>(31 - __builtin_clz(n));
  // update to match bit width
#else
  int result = 0;
  while (n >>= 1) {
    ++result;
  }
  return result;
#endif
}

using point_t = core::Point<coord_t>; ///< Data type for a 3D point

// Coordinate type must be floating point (float, double, long double)
static_assert(std::is_floating_point_v<coord_t>);

// Distance type must be floating point (float, double, long double)
static_assert(std::is_floating_point_v<dist_t>);

} // namespace pcr

#endif // PRELUDE_HPP