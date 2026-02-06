#define CATCH_CONFIG_MAIN
#include <algorithm>
#include <catch2/catch_all.hpp>
#include <vector>

#include "pcr/core/point_cloud.hpp"
#include "pcr/spatial/kd_tree.hpp"

// Helper to create a simple test cloud
pcr::core::PointCloud create_test_cloud() {
  pcr::core::PointCloud cloud;
  cloud.add({static_cast<pcr::coord_t>(0.0), static_cast<pcr::coord_t>(0.0),
             static_cast<pcr::coord_t>(0.0)});
  cloud.add({static_cast<pcr::coord_t>(1.0), static_cast<pcr::coord_t>(0.0),
             static_cast<pcr::coord_t>(0.0)});
  cloud.add({static_cast<pcr::coord_t>(10.0), static_cast<pcr::coord_t>(10.0),
             static_cast<pcr::coord_t>(10.0)});
  return cloud;
}


using point_type = pcr::point_t;

TEST_CASE("KdTree: Construction and Custom Dimensions", "[spatial]") {

  SECTION("Default constructor functions") {
    pcr::spatial::KdTree tree;
    pcr::core::PointCloud cloud = create_test_cloud();
    REQUIRE_NOTHROW(tree.build_index(&cloud));
  }

  SECTION("Parametrized constructor with custom dimensions)") {
    std::vector<pcr::coord_t point_type::*> dims = {&point_type::x,
                                                    &point_type::y};
    REQUIRE_NOTHROW(pcr::spatial::KdTree(dims));
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
  std::vector<pcr::dist_t> dists;

  SECTION("KNN on empty tree returns nothing") {
    tree.knn_search(query, 5, indices, dists);
    CHECK(indices.empty());
    CHECK(dists.empty());
  }

  SECTION("Radius search on empty tree returns nothing") {
    tree.radius_search(query, static_cast<pcr::dist_t>(10.0), indices, dists);
    CHECK(indices.empty());
  }
}

TEST_CASE("KdTree: KNN Search Logic", "[spatial]") {
  pcr::spatial::KdTree tree;
  auto cloud = create_test_cloud();
  tree.build_index(&cloud);

  std::vector<pcr::point_idx> indices;
  std::vector<pcr::dist_t> dists;

  SECTION("K=1 finds the exact point if it exists") {
    point_type query{static_cast<pcr::coord_t>(10.0),
                     static_cast<pcr::coord_t>(10.0),
                     static_cast<pcr::coord_t>(10.0)}; // Matches point tree 2
    tree.knn_search(query, 1, indices, dists);

    REQUIRE(indices.size() == 1);
    CHECK(indices[0] == 2);
    CHECK(dists[0] == Catch::Approx(static_cast<pcr::coord_t>(0.0)));
  }

  SECTION("K=2 finds multiple points in order of distance") {
    point_type query{static_cast<pcr::coord_t>(0.1),
                     static_cast<pcr::coord_t>(0.0),
                     static_cast<pcr::coord_t>(0.0)};
    tree.knn_search(query, 2, indices, dists);

    REQUIRE(indices.size() == 2);
    CHECK(indices[0] == 0); // Origin is 0.1 away
    CHECK(indices[1] == 1); // {1,0,0} is 0.9 away
    CHECK(dists[0] < dists[1]);
  }

  SECTION("Requesting K > total points returns all points available") {
    point_type query{static_cast<pcr::coord_t>(0.0),
                     static_cast<pcr::coord_t>(0.0),
                     static_cast<pcr::coord_t>(0.0)};
    tree.knn_search(query, 100, indices, dists);
    CHECK(indices.size() == cloud.size());
  }
}

TEST_CASE("KdTree: Radius Search Logic", "[spatial]") {
  pcr::spatial::KdTree tree;
  auto cloud = create_test_cloud();
  tree.build_index(&cloud);

  std::vector<pcr::point_idx> indices;
  std::vector<pcr::dist_t> dists;

  SECTION("Small radius finds only the closest point") {
    point_type query{static_cast<pcr::coord_t>(0.1),
                     static_cast<pcr::coord_t>(0.0),
                     static_cast<pcr::coord_t>(0.0)};
    pcr::dist_t radius = 0.5; // Should only catch origin
    tree.radius_search(query, radius, indices, dists);

    CHECK(indices.size() == 1);
    CHECK(indices[0] == 0);
  }

  SECTION("Large radius finds all points") {
    point_type query{static_cast<pcr::coord_t>(0.0),
                     static_cast<pcr::coord_t>(0.0),
                     static_cast<pcr::coord_t>(0.0)};
    pcr::dist_t radius = 100.0;
    tree.radius_search(query, radius, indices, dists);

    CHECK(indices.size() == cloud.size());
  }

  SECTION("Distances returned are squared") {
    point_type query{static_cast<pcr::coord_t>(5.0),
                     static_cast<pcr::coord_t>(0.0),
                     static_cast<pcr::coord_t>(0.0)};
    tree.radius_search(query, static_cast<pcr::dist_t>(10.0), indices, dists);

    // Point {0,0,0} is distance 5 away. Squared distance should be 25.
    auto it = std::find(indices.begin(), indices.end(), 0);
    REQUIRE(it != indices.end());
    auto found_idx = static_cast<size_t>(std::distance(indices.begin(), it));
    CHECK(dists[found_idx] == Catch::Approx(25.0));
  }
}
