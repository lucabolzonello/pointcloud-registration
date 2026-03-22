// This is pretty much the same as test_kd_tree.cpp, but with just a brute force test-oracle style check.
// I don't expect it to be efficient, so don't know if it should be part of a full regression-test suite that gets run often.
// So that's why I have separated it here
#define CATCH_CONFIG_MAIN
#include <algorithm>
#include <catch2/catch_all.hpp>
#include <iostream>
#include <vector>

#include "pcr/core/point_cloud.hpp"
#include "pcr/spatial/kd_tree.hpp"
#include "pcr/io/ply.hpp"
#include "../tests/spatial/kd_tree_helpers.hpp"


#define STRING(x) #x
#define XSTRING(x) STRING(x)

std::string bunny_045{"/bunny/data/ear_back.ply"};


TEST_CASE("KNN Search, brute force", "[spatial]") {
  SECTION("bun045.ply test case") {
    std::string filepath = XSTRING(DATASETS_PATH) + bunny_045;

    pcr::core::PointCloud pc = pcr::io::ply::read_file(filepath);

    pcr::spatial::KdTree kd;
    kd.build_index(&pc);
    pcr::point_idx k = 10;

    for (auto point : pc) {
      std::vector<pcr::point_idx> indices;
      std::vector<pcr::dist_t> distances;

      std::vector<pcr::point_idx> oracle_indices;
      std::vector<pcr::dist_t> oracle_distances;

      kd.knn_search(point, k, indices, distances);

      brute_force_knn(pc, point, k, oracle_indices, oracle_distances);
      CHECK(compare_results(pc, indices, distances, oracle_indices,
        oracle_distances));
    }

  }
}

TEST_CASE("Radius Search, brute force", "[spatial]") {
  SECTION("bun045.ply test case") {
    std::string filepath = XSTRING(DATASETS_PATH) + bunny_045;

    pcr::core::PointCloud pc = pcr::io::ply::read_file(filepath);

    pcr::spatial::KdTree kd;
    kd.build_index(&pc);
    pcr::point_idx k = 10;

    for (auto point : pc) {
      std::vector<pcr::point_idx> indices;
      std::vector<pcr::dist_t> distances;

      std::vector<pcr::point_idx> oracle_indices;
      std::vector<pcr::dist_t> oracle_distances;

      const pcr::dist_t search_radius = 0.3f;

      kd.radius_search(point, search_radius, indices, distances);

      brute_force_radius(pc, point, search_radius, oracle_indices,
                         oracle_distances);
      CHECK(compare_results(pc, indices, distances, oracle_indices,
        oracle_distances));

    }

  }
}
