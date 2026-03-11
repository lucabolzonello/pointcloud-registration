#define CATCH_CONFIG_MAIN
#include <algorithm>
#include <catch2/catch_all.hpp>
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
  return cloud;
}


using point_type = pcr::point_t;

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
  auto orig_cloud = cloud; // Copy for the convenience of testing
  tree.build_index(&cloud);

  std::vector<pcr::point_idx> k_nearest_indices;
  std::vector<pcr::dist_t> k_nearest_distances;

  SECTION("K=1 finds the exact point if it exists") {
    point_type query{static_cast<pcr::coord_t>(36.2),
                     static_cast<pcr::coord_t>(669.0),
                     static_cast<pcr::coord_t>(21.0)};
    tree.knn_search(query, 1, k_nearest_indices, k_nearest_distances);

    REQUIRE(k_nearest_indices.size() == 1);
    REQUIRE(k_nearest_distances.size() == 1);

    pcr::point_t near_point = orig_cloud[4];
    CHECK((near_point.x == Catch::Approx(static_cast<pcr::coord_t>(36.2)) &&
           near_point.y == Catch::Approx(static_cast<pcr::coord_t>(669.0)) &&
           near_point.z == Catch::Approx(static_cast<pcr::coord_t>(21.0))));

    CHECK(k_nearest_distances[0] ==
          Catch::Approx(static_cast<pcr::coord_t>(0.0)));

  }

  }

  SECTION("K=2 finds the 2 nearest neighbours") {
    point_type query{static_cast<pcr::coord_t>(4.3),
                     static_cast<pcr::coord_t>(28.95),
                     static_cast<pcr::coord_t>(15.4)};
    tree.knn_search(query, 2, k_nearest_indices, k_nearest_distances);

    REQUIRE(k_nearest_indices.size() == 2);
    REQUIRE(k_nearest_distances.size() == 2);

    std::vector<int> order{k_nearest_distances[0] <= k_nearest_distances[1] ? 0
                                                                               : 1,
                              k_nearest_distances[0] <= k_nearest_distances[1] ? 1
                                                                               : 0};

    pcr::point_t first_point = orig_cloud[1];
    pcr::point_t second_point = orig_cloud[2];

    CHECK(first_point.x == Catch::Approx(static_cast<pcr::coord_t>(4.4)));
    CHECK(first_point.y == Catch::Approx(static_cast<pcr::coord_t>(29.0)));
    CHECK(first_point.z == Catch::Approx(static_cast<pcr::coord_t>(15.5)));

    CHECK(second_point.x == Catch::Approx(static_cast<pcr::coord_t>(4.5)));
    CHECK(second_point.y == Catch::Approx(static_cast<pcr::coord_t>(28.8)));
    CHECK(second_point.z == Catch::Approx(static_cast<pcr::coord_t>(16.0)));

    CHECK(k_nearest_distances[order[0]] ==
          Catch::Approx(static_cast<pcr::coord_t>(0.0225)));
    CHECK(k_nearest_distances[order[1]] ==
          Catch::Approx(static_cast<pcr::coord_t>(0.4225)));
  }

  SECTION("Requesting K > total points returns all points available") {
    point_type query{static_cast<pcr::coord_t>(0.0),
                     static_cast<pcr::coord_t>(0.0),
                     static_cast<pcr::coord_t>(0.0)};
    tree.knn_search(query, 100, k_nearest_indices, k_nearest_distances);
    CHECK(k_nearest_indices.size() == cloud.size());
    CHECK(k_nearest_distances.size() == cloud.size());
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
    tree.radius_search(query, radius, indices, distances_squared);

    REQUIRE(indices.size() == 1);
    REQUIRE(distances_squared.size() == 1);
    CHECK(cloud[indices[0]].x == Catch::Approx(static_cast<pcr::coord_t>(36.2)));
    CHECK(cloud[indices[0]].y == Catch::Approx(static_cast<pcr::coord_t>(669.0)));
    CHECK(cloud[indices[0]].z == Catch::Approx(static_cast<pcr::coord_t>(21.0)));
  }

  SECTION("Large radius finds all points") {
    point_type query{static_cast<pcr::coord_t>(0.0),
                     static_cast<pcr::coord_t>(0.0),
                     static_cast<pcr::coord_t>(0.0)};
    pcr::dist_t radius = 10000.0;
    tree.radius_search(query, radius, indices, distances_squared);

    CHECK(indices.size() == cloud.size());
    CHECK(distances_squared.size() == cloud.size());
  }

  SECTION("Distances returned are squared") {
    point_type query{static_cast<pcr::coord_t>(36.3),
                     static_cast<pcr::coord_t>(669.0),
                     static_cast<pcr::coord_t>(21.0)};
    tree.radius_search(query, static_cast<pcr::dist_t>(1.0), indices,
                       distances_squared);

    auto it = std::find_if(indices.begin(), indices.end(), [&](pcr::point_idx idx) {
      const pcr::point_t point = cloud[idx];
      return point.x == Catch::Approx(static_cast<pcr::coord_t>(36.2)) &&
             point.y == Catch::Approx(static_cast<pcr::coord_t>(669.0)) &&
             point.z == Catch::Approx(static_cast<pcr::coord_t>(21.0));
    });

    REQUIRE(it != indices.end());
    auto found_idx = static_cast<size_t>(std::distance(indices.begin(), it));
    CHECK(distances_squared[found_idx] ==
          Catch::Approx(static_cast<pcr::dist_t>(0.01)));
  }
}