#ifndef POINTCLOUD_REGISTRATION_KD_TREE_HELPERS_HPP
#define POINTCLOUD_REGISTRATION_KD_TREE_HELPERS_HPP
#include <vector>
#include "pcr/prelude.hpp"
#include "pcr/core/point_cloud.hpp"

// Brute force KNN search for verification
void brute_force_knn(const pcr::core::PointCloud &cloud,
                     const pcr::core::Point<pcr::dist_t> &query,
                     size_t k, std::vector<pcr::point_idx> &indices,
                     std::vector<pcr::dist_t> &distances) {
  indices.clear();
  distances.clear();

  std::vector<std::pair<pcr::dist_t, pcr::point_idx>> results;
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto &point = cloud[i];
    pcr::dist_t dx = point.x - query.x;
    pcr::dist_t dy = point.y - query.y;
    pcr::dist_t dz = point.z - query.z;
    pcr::dist_t dist_sq = dx * dx + dy * dy + dz * dz;
    results.emplace_back(dist_sq, i);
  }

  const size_t num_results = std::min(k, results.size());
  std::nth_element(results.begin(), results.begin() + num_results,
                   results.end());

  for (size_t i = 0; i < num_results; ++i) {
    distances.push_back(results[i].first);
    indices.push_back(results[i].second);
  }
}


// Brute force radius search for verification
void brute_force_radius(const pcr::core::PointCloud &cloud,
                        const pcr::core::Point<pcr::dist_t> &query,
                        pcr::dist_t radius,
                        std::vector<pcr::point_idx> &indices,
                        std::vector<pcr::dist_t> &distances) {
  indices.clear();
  distances.clear();

  pcr::dist_t radius_sq = radius * radius;
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto &point = cloud[i];
    pcr::dist_t dx = point.x - query.x;
    pcr::dist_t dy = point.y - query.y;
    pcr::dist_t dz = point.z - query.z;
    pcr::dist_t dist_sq = dx * dx + dy * dy + dz * dz;

    if (dist_sq <= radius_sq) {
      distances.push_back(dist_sq);
      indices.push_back(i);
    }
  }
}

// Compare two search results (sorts and checks if they match)
bool compare_results(const pcr::core::PointCloud &cloud,
                     std::vector<pcr::point_idx> indices1,
                     std::vector<pcr::dist_t> distances1,
                     std::vector<pcr::point_idx> indices2,
                     std::vector<pcr::dist_t> distances2,
                     pcr::dist_t tolerance = 1e-6) {
  if (indices1.size() != indices2.size() || distances1.size() != distances2.
      size()) {
    std::cout << "Different sizes" << std::endl;
    std::cout << "Indices 1 size: " << indices1.size() << ", Indices2 size: "
        << indices2.size() << std::endl;
    return false;
  }

  std::sort(distances1.begin(), distances1.end());
  std::sort(distances2.begin(), distances2.end());

  // Compare sorted results
  for (size_t i = 0; i < distances1.size(); ++i) {
    // Compare distances
    if (std::abs(distances1[i] - distances2[i]) > tolerance) {
      std::cout << "Different distances" << std::endl;
      return false;
    }
  }

  return
      true;
}
#endif //POINTCLOUD_REGISTRATION_KD_TREE_HELPERS_HPP