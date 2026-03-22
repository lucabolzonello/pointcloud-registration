#define CATCH_CONFIG_MAIN
#include <algorithm>
#include <catch2/catch_all.hpp>
#include <iostream>
#include <vector>

#include "pcr/core/point_cloud.hpp"
#include "pcr/spatial/kd_tree.hpp"

// Helper to create a simple test cloud
pcr::core::PointCloud create_test_cloud() {
  pcr::core::PointCloud cloud;
  cloud.add({static_cast<pcr::coord_t>(1.2), static_cast<pcr::coord_t>(29.1),
             static_cast<pcr::coord_t>(10.0)});
  cloud.add({static_cast<pcr::coord_t>(4.4), static_cast<pcr::coord_t>(29.0),
             static_cast<pcr::coord_t>(15.5)});
  cloud.add({static_cast<pcr::coord_t>(4.5), static_cast<pcr::coord_t>(28.8),
             static_cast<pcr::coord_t>(16.0)});
  cloud.add({static_cast<pcr::coord_t>(21.2), static_cast<pcr::coord_t>(19.0),
             static_cast<pcr::coord_t>(12.0)});
  cloud.add({static_cast<pcr::coord_t>(36.2), static_cast<pcr::coord_t>(669.0),
             static_cast<pcr::coord_t>(21.0)});
  cloud.add({static_cast<pcr::coord_t>(56.8), static_cast<pcr::coord_t>(8093.0),
             static_cast<pcr::coord_t>(10.0)});
  cloud.add({static_cast<pcr::coord_t>(59.7), static_cast<pcr::coord_t>(8090.0),
             static_cast<pcr::coord_t>(-10.0)});
  cloud.add({static_cast<pcr::coord_t>(9.0), static_cast<pcr::coord_t>(9.0),
             static_cast<pcr::coord_t>(10.0)});
  cloud.add({static_cast<pcr::coord_t>(-10.0), static_cast<pcr::coord_t>(43.0),
             static_cast<pcr::coord_t>(10.0)});
  cloud.add({static_cast<pcr::coord_t>(-90.2), static_cast<pcr::coord_t>(-20.0),
             static_cast<pcr::coord_t>(-100.0)});
  cloud.add({static_cast<pcr::coord_t>(-90.0), static_cast<pcr::coord_t>(-21.5),
             static_cast<pcr::coord_t>(-100.8)});
  cloud.add({static_cast<pcr::coord_t>(5.78), static_cast<pcr::coord_t>(56.2),
             static_cast<pcr::coord_t>(20.0)});
  cloud.add({static_cast<pcr::coord_t>(100.2),
             static_cast<pcr::coord_t>(-100.0),
             static_cast<pcr::coord_t>(25.0)});

  return cloud;
}

using point_type = pcr::point_t;

// Brute force KNN search for verification
void brute_force_knn(const pcr::core::PointCloud& cloud, const point_type& query,
                     size_t k, std::vector<pcr::point_idx>& indices,
                     std::vector<pcr::dist_t>& distances) {
  indices.clear();
  distances.clear();

  std::vector<std::pair<pcr::dist_t, pcr::point_idx>> results;
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& point = cloud[i];
    pcr::dist_t dx = point.x - query.x;
    pcr::dist_t dy = point.y - query.y;
    pcr::dist_t dz = point.z - query.z;
    pcr::dist_t dist_sq = dx * dx + dy * dy + dz * dz;
    results.emplace_back(dist_sq, i);
  }

  std::sort(results.begin(), results.end());

  size_t num_results = std::min(k, results.size());
  for (size_t i = 0; i < num_results; ++i) {
    distances.push_back(results[i].first);
    indices.push_back(results[i].second);
  }
}

// Brute force radius search for verification
void brute_force_radius(const pcr::core::PointCloud& cloud, const point_type& query,
                        pcr::dist_t radius, std::vector<pcr::point_idx>& indices,
                        std::vector<pcr::dist_t>& distances) {
  indices.clear();
  distances.clear();

  pcr::dist_t radius_sq = radius * radius;
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& point = cloud[i];
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
bool compare_results(const pcr::core::PointCloud& cloud,
                    std::vector<pcr::point_idx> indices1,
                    std::vector<pcr::dist_t> distances1,
                    std::vector<pcr::point_idx> indices2,
                    std::vector<pcr::dist_t> distances2,
                    pcr::dist_t tolerance = 1e-6) {
  if (indices1.size() != indices2.size() || distances1.size() != distances2.size()) {
    return false;
  }

  // Create pairs and sort both
  std::vector<std::pair<pcr::dist_t, pcr::point_idx>> pairs1, pairs2;
  for (size_t i = 0; i < indices1.size(); ++i) {
    pairs1.emplace_back(distances1[i], indices1[i]);
    pairs2.emplace_back(distances2[i], indices2[i]);
  }

  std::sort(pairs1.begin(), pairs1.end());
  std::sort(pairs2.begin(), pairs2.end());

  // Compare sorted results
  for (size_t i = 0; i < pairs1.size(); ++i) {
    // Compare distances
    if (std::abs(pairs1[i].first - pairs2[i].first) > tolerance) {
      printf("Different distances");
      return false;
    }

    // Compare points by x, y, z values
    const auto& pt1 = cloud[pairs1[i].second];
    const auto& pt2 = cloud[pairs2[i].second];
    if (std::abs(pt1.x - pt2.x) > tolerance ||
        std::abs(pt1.y - pt2.y) > tolerance ||
        std::abs(pt1.z - pt2.z) > tolerance) {
      return false;
    }
  }

  return true;
}

TEST_CASE("KdTree: Construction and Custom Dimensions", "[spatial]") {

  SECTION("Default constructor functions") {
    pcr::spatial::KdTree tree;
    pcr::core::PointCloud cloud = create_test_cloud();
    REQUIRE_NOTHROW(tree.build_index(&cloud));
  }
}

TEST_CASE("KdTree: Empty Cloud Behavior", "[spatial]") {
  pcr::spatial::KdTree tree;
  pcr::core::PointCloud empty_cloud;
  tree.build_index(&empty_cloud);

  point_type query{static_cast<pcr::coord_t>(1.0),
                   static_cast<pcr::coord_t>(1.0),
                   static_cast<pcr::coord_t>(1.0)};
  std::vector<pcr::point_idx> indices;
  std::vector<pcr::dist_t> distances_squared;

  SECTION("KNN on empty tree returns nothing") {
    tree.knn_search(query, 5, indices, distances_squared);
    CHECK(indices.empty());
    CHECK(distances_squared.empty());
  }

  SECTION("Radius search on empty tree returns nothing") {
    tree.radius_search(query, static_cast<pcr::dist_t>(10.0), indices,
                       distances_squared);
    CHECK(indices.empty());
    CHECK(distances_squared.empty());
  }
}

TEST_CASE("KdTree: KNN Search Logic", "[spatial]") {
  pcr::spatial::KdTree tree;
  auto cloud = create_test_cloud();
  tree.build_index(&cloud);

  std::vector<pcr::point_idx> k_nearest_indices;
  std::vector<pcr::dist_t> k_nearest_distances;

  SECTION("K=1 finds the exact point if it exists") {
    point_type query{static_cast<pcr::coord_t>(36.2),
                     static_cast<pcr::coord_t>(669.0),
                     static_cast<pcr::coord_t>(21.0)};

    std::vector<pcr::point_idx> brute_indices;
    std::vector<pcr::dist_t> brute_distances;
    brute_force_knn(cloud, query, 1, brute_indices, brute_distances);

    tree.knn_search(query, 1, k_nearest_indices, k_nearest_distances);

    CHECK(compare_results(cloud, k_nearest_indices, k_nearest_distances,
      brute_indices, brute_distances));
  }

  SECTION("K = 1 finds the nearest point") {
    point_type query{static_cast<pcr::coord_t>(4.4),
                     static_cast<pcr::coord_t>(29.1),
                     static_cast<pcr::coord_t>(16.0)};

    std::vector<pcr::point_idx> brute_indices;
    std::vector<pcr::dist_t> brute_distances;
    brute_force_knn(cloud, query, 1, brute_indices, brute_distances);

    tree.knn_search(query, 1, k_nearest_indices, k_nearest_distances);

    CHECK(compare_results(cloud, k_nearest_indices, k_nearest_distances,
      brute_indices, brute_distances));
  }

  SECTION("K=2 finds the 2 nearest neighbours") {
    point_type query{static_cast<pcr::coord_t>(4.3),
                     static_cast<pcr::coord_t>(28.95),
                     static_cast<pcr::coord_t>(15.4)};

    std::vector<pcr::point_idx> brute_indices;
    std::vector<pcr::dist_t> brute_distances;
    brute_force_knn(cloud, query, 2, brute_indices, brute_distances);

    tree.knn_search(query, 2, k_nearest_indices, k_nearest_distances);

    CHECK(compare_results(cloud, k_nearest_indices, k_nearest_distances,
      brute_indices, brute_distances));
  }

  SECTION("K = 8 finds the 8 nearest neighbours") {
    point_type query{static_cast<pcr::coord_t>(4.32),
                     static_cast<pcr::coord_t>(28.950),
                     static_cast<pcr::coord_t>(35.402)};

    std::vector<pcr::point_idx> brute_indices;
    std::vector<pcr::dist_t> brute_distances;
    brute_force_knn(cloud, query, 8, brute_indices, brute_distances);

    tree.knn_search(query, 8, k_nearest_indices, k_nearest_distances);

    CHECK(compare_results(cloud, k_nearest_indices, k_nearest_distances,
      brute_indices, brute_distances));
  }

  SECTION("Requesting K > total points returns all points available") {
    point_type query{static_cast<pcr::coord_t>(0.0),
                     static_cast<pcr::coord_t>(0.0),
                     static_cast<pcr::coord_t>(0.0)};

    std::vector<pcr::point_idx> brute_indices;
    std::vector<pcr::dist_t> brute_distances;
    brute_force_knn(cloud, query, 100, brute_indices, brute_distances);

    tree.knn_search(query, 100, k_nearest_indices, k_nearest_distances);

    CHECK(compare_results(cloud, k_nearest_indices, k_nearest_distances,
      brute_indices, brute_distances));
  }
}

TEST_CASE("KdTree: Radius Search Logic", "[spatial]") {
  pcr::spatial::KdTree tree;
  auto cloud = create_test_cloud();
  tree.build_index(&cloud);

  std::vector<pcr::point_idx> indices;
  std::vector<pcr::dist_t> distances_squared;

  SECTION("Small radius finds only the closest point") {
    point_type query{static_cast<pcr::coord_t>(36.3),
                     static_cast<pcr::coord_t>(669.0),
                     static_cast<pcr::coord_t>(21.0)};
    pcr::dist_t radius = 0.5;

    std::vector<pcr::point_idx> brute_indices;
    std::vector<pcr::dist_t> brute_distances;
    brute_force_radius(cloud, query, radius, brute_indices, brute_distances);

    tree.radius_search(query, radius, indices, distances_squared);

    CHECK(compare_results(cloud, indices, distances_squared,
      brute_indices, brute_distances));
  }

  SECTION("Large radius finds all points") {
    point_type query{static_cast<pcr::coord_t>(0.0),
                     static_cast<pcr::coord_t>(0.0),
                     static_cast<pcr::coord_t>(0.0)};
    pcr::dist_t radius = 10000.0;

    std::vector<pcr::point_idx> brute_indices;
    std::vector<pcr::dist_t> brute_distances;
    brute_force_radius(cloud, query, radius, brute_indices, brute_distances);

    tree.radius_search(query, radius, indices, distances_squared);

    CHECK(compare_results(cloud, indices, distances_squared,
      brute_indices, brute_distances));
  }

  SECTION("Distances returned are squared") {
    point_type query{static_cast<pcr::coord_t>(36.3),
                     static_cast<pcr::coord_t>(669.0),
                     static_cast<pcr::coord_t>(21.0)};
    pcr::dist_t radius = 1.0;

    std::vector<pcr::point_idx> brute_indices;
    std::vector<pcr::dist_t> brute_distances;
    brute_force_radius(cloud, query, radius, brute_indices, brute_distances);

    tree.radius_search(query, radius, indices, distances_squared);

    CHECK(compare_results(cloud, indices, distances_squared,
      brute_indices, brute_distances));
  }
}