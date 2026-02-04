/**
 * @file ply.hpp
 * @brief PLY file format I/O for point clouds
 */

#ifndef PLY_HPP
#define PLY_HPP

#include "pcr/core/point_cloud.hpp"
#include <cstring>
#include <string>

/**
 * @namespace pcr::io::ply
 * @brief Interface for reading and writing PLY files
 */
namespace pcr::io::ply {

/**
 * @brief Read PLY file into PointCloud object
 *
 * Parses PLY file format  and constructs a PointCloud containing the vertex
 * data.
 *
 * @param path_to_file Path to PLY file to read
 * @return PointCloud containing all vertices from file
 *
 * @throws std::runtime_error if file cannot be opened
 * @throws std::runtime_error if file format is invalid
 * @throws std::runtime_error if file parsing fails
 */
pcr::core::PointCloud read_file(const std::string &path_to_file);

/**
 * @brief Write PointCloud object to PLY file
 *
 * Serializes point cloud to PLY file format. Uses binary encoding
 * by default for better performance and smaller file size.
 *
 * @param path_to_file Path where PLY file will be written
 * @param src_point_cloud Point cloud to serialize
 * @param binary If true, writes binary PLY; if false, writes ASCII PLY
 *
 * @throws std::runtime_error if file cannot be created
 * @throws std::runtime_error if write operation fails
 */
void write_file(const std::string &path_to_file,
                const pcr::core::PointCloud &src_point_cloud,
                const bool binary = true);

} // namespace pcr::io::ply

#endif // PLY_HPP