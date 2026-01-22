#ifndef PLY_HPP
#define PLY_HPP

#include "pcr/core/point_cloud.hpp"
#include <cstring>

namespace pcr::io::ply {

// read_file(path_to_file, preload_into_memory), Read PLY file into PointCloud
// object
// throws std::exception on failure.
pcr::core::PointCloud read_file(const std::string &path_to_file);

// write_file(path_to_file, src_point_cloud, ascii), Write PointCloud object
// to PLY file
// Binary by default for performance
// throws std::runtime_error on failure.
void write_file(const std::string &path_to_file,
                const pcr::core::PointCloud &src_point_cloud,
                const bool binary = true);

} // namespace pcr::io::ply

#endif