/**
 * @file point_cloud.hpp
 * @brief Point cloud container with integrated spatial metadata
 */

#ifndef POINT_CLOUD_HPP
#define POINT_CLOUD_HPP

#include "bounding_box.hpp"
#include "pcr/prelude.hpp"
#include "point.hpp"

#include <cstddef>
#include <vector>

/**
 * @namespace pcr::core
 * @brief Core data structures for point cloud registration
 */
namespace pcr::core {

/**
 * @brief Container for 3D point cloud data
 *
 * Manages and owns a collection of 3D points with integrated bounding box
 * maintenance.
 */
class PointCloud {
public:
  using point_iterator =
      std::vector<pcr::point_t>::iterator; ///< Mutable iterator type
  using const_point_iterator =
      std::vector<pcr::point_t>::const_iterator; ///< Const iterator type

  /**
   * @brief Default constructor - creates empty point cloud
   *
   * Time Complexity: O(1)
   */
  PointCloud();

  /**
   * @brief Construct from iterator range
   *
   * Creates point cloud from all points in [begin, end).
   *
   * Time Complexity: O(n) where n = distance(begin, end)
   *
   * @tparam Iterator Input iterator type that yields point_type
   * @param begin Iterator to first point
   * @param end Iterator one past last point
   */
  template <typename Iterator>
  PointCloud(Iterator const begin, Iterator const end)
      : m_bounding_box(), data() {
    data.reserve(end - begin);
    for (Iterator point = begin; point != end; ++point) {
      this->add(*point);
    }
  }

  /**
   * @brief Get number of points in cloud
   *
   * Time Complexity: O(1)
   *
   * @return Number of points currently stored
   */
  [[nodiscard]] std::size_t size() const noexcept;

  /**
   * @brief Reserve memory for n points
   *
   * Pre-allocates storage to avoid reallocation during incremental
   * point addition. Does not change size().
   *
   * Time Complexity: O(n) if reallocation needed
   *
   * @param n Number of points to reserve space for
   */
  void reserve(std::size_t n);

  /**
   * @brief Check if point cloud is empty
   *
   * Time Complexity: O(1)
   *
   * @return true if cloud contains no points, false otherwise
   */
  [[nodiscard]] bool is_empty() const noexcept;

  /**
   * @brief Add a single point to the cloud
   *
   * Appends point and updates bounding box metadata.
   *
   * Time Complexity: O(1) amortized, O(n) worst case if reallocation needed
   *
   * @param p Point to add
   */
  void add(pcr::point_t p);

  /**
   * @brief Add points from iterator range
   *
   * Adds all points in [begin, end) to the cloud.
   * Source container remains unchanged.
   *
   * Time Complexity: O(n) amortized where n = distance(begin, end)
   *
   * @tparam Iterator Input iterator type yielding point_type
   * @param begin Iterator to first point
   * @param end Iterator one past last point
   */
  template <typename Iterator>
  void add(Iterator const begin, Iterator const end) {
    for (Iterator point = begin; point != end; ++point) {
      this->add(*point);
    }
  }

  /**
   * @brief Get iterator to first point
   *
   * Time Complexity: O(1)
   *
   * @return Iterator to beginning of point sequence
   */
  [[nodiscard]] point_iterator begin();

  /**
   * @brief Get const iterator to first point
   *
   * Time Complexity: O(1)
   *
   * @return Const iterator to beginning of point sequence
   */
  [[nodiscard]] const_point_iterator begin() const;

  /**
   * @brief Get const iterator to first point
   *
   * Time Complexity: O(1)
   *
   * @return Const iterator to beginning of point sequence
   */
  [[nodiscard]] const_point_iterator cbegin() const;

  /**
   * @brief Get iterator to one past last point
   *
   * Time Complexity: O(1)
   *
   * @return Iterator to end of point sequence
   */
  [[nodiscard]] point_iterator end();

  /**
   * @brief Get const iterator to one past last point
   *
   * Time Complexity: O(1)
   *
   * @return Const iterator to end of point sequence
   */
  [[nodiscard]] const_point_iterator end() const;

  /**
   * @brief Get const iterator to one past last point
   *
   * Time Complexity: O(1)
   *
   * @return Const iterator to end of point sequence
   */
  [[nodiscard]] const_point_iterator cend() const;

  /**
   * @brief Get copy of current bounding box
   *
   * Time Complexity: O(1)
   *
   * @return BoundingBox encompassing all points in cloud
   */
  [[nodiscard]] BoundingBox<pcr::coord_t> get_bounding_box() const;

  /**
   * @brief Access point by index
   *
   * Time Complexity: O(1)
   *
   * @param index Zero-based point index
   * @return Reference to point at specified index
   *
   * @warning No bounds checking for efficiency reasons, undefined behavior if
   * index >= size()
   */
  [[nodiscard]] inline pcr::point_t &operator[](size_t index) {
    return data[index];
  }

  /**
   * @brief Access point by index (const version)
   *
   * Time Complexity: O(1)
   *
   * @param index Zero-based point index
   * @return Const reference to point at specified index
   *
   * @warning No bounds checking for efficiency reasons, undefined behavior if
   * index >= size()
   */
  [[nodiscard]] inline const pcr::point_t &operator[](size_t index) const {
    return data[index];
  }

private:
  BoundingBox<pcr::coord_t> m_bounding_box; ///< Spatial Bounding Box
  std::vector<pcr::point_t> data;           ///< Point storage
};

} // namespace pcr::core

#endif // POINT_CLOUD_HPP