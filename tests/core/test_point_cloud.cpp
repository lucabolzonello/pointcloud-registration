#define CATCH_CONFIG_MAIN
#include <array>
#include <catch2/catch_all.hpp>
#include <limits>

#include "pcr/core/point.hpp"
#include "pcr/core/point_cloud.hpp"
#include "pcr/prelude.hpp"

// are_points_equal(), compare points by value of x,y,z coordinates
// Note: This is just for unit testing purposes, due to float rounding
// complications it is inappropriate to use in actual computations.
template <typename T>
bool are_points_equal(const pcr::core::Point<T> &lhs,
                      const pcr::core::Point<T> &rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

using point_value_type = pcr::coord_t;

TEST_CASE("Default initialize PointCloud", "[constructor]") {
  pcr::core::PointCloud pc{};

  SECTION("size() works correctly when empty") { CHECK(pc.size() == 0); }

  SECTION("Iterators are set correctly when empty") {
    CHECK(pc.begin() == pc.end());
  }
}

TEST_CASE("Iterator based PointCloud constructor", "[constructor]") {
  pcr::core::Point<point_value_type> p1(static_cast<point_value_type>(1.5),
                                        static_cast<point_value_type>(2.5),
                                        static_cast<point_value_type>(3.5));

  pcr::core::Point<point_value_type> p2(static_cast<point_value_type>(60.5),
                                        static_cast<point_value_type>(80.5),
                                        static_cast<point_value_type>(0.5));

  pcr::core::Point<point_value_type> p3(static_cast<point_value_type>(0.0),
                                        static_cast<point_value_type>(0.0),
                                        static_cast<point_value_type>(0.0));

  SECTION("Construct from empty Iterator") {
    std::array<pcr::core::Point<point_value_type>, 0> points{};
    pcr::core::PointCloud pc(points.begin(), points.end());

    CHECK(pc.size() == 0);
    CHECK(pc.empty());
  }

  SECTION("Construct from single element Iterator") {
    std::array<pcr::core::Point<point_value_type>, 1> points{p1};
    pcr::core::PointCloud pc(points.begin(), points.end());

    CHECK(pc.size() == 1);
    CHECK_FALSE(pc.empty());
  }

  SECTION("Construct form multi element Iterator") {
    std::array<pcr::core::Point<point_value_type>, 3> points{p1, p2, p3};
    pcr::core::PointCloud pc(points.begin(), points.end());

    CHECK(pc.size() == 3);
    CHECK_FALSE(pc.empty());
  }
}

TEST_CASE("Add point", "[add]") {
  pcr::core::Point<point_value_type> p1(static_cast<point_value_type>(1.5),
                                        static_cast<point_value_type>(2.5),
                                        static_cast<point_value_type>(3.5));

  pcr::core::Point<point_value_type> p2(static_cast<point_value_type>(60.5),
                                        static_cast<point_value_type>(80.5),
                                        static_cast<point_value_type>(0.5));

  pcr::core::Point<point_value_type> p3(static_cast<point_value_type>(0.0),
                                        static_cast<point_value_type>(0.0),
                                        static_cast<point_value_type>(0.0));

  SECTION("Add single point") {
    pcr::core::PointCloud pc{};
    pc.push_back(p1);

    CHECK(pc.size() == 1);
    CHECK_FALSE(pc.empty());
  }

  SECTION("Add multiple points") {
    pcr::core::PointCloud pc{};
    pc.push_back(p1);
    pc.push_back(p2);
    pc.push_back(p3);

    CHECK(pc.size() == 3);
    CHECK_FALSE(pc.empty());
  }
}

// TEST_CASE("Add point from iterator", "[add]") {
//   pcr::core::Point<point_value_type> p1(static_cast<point_value_type>(1.5),
//                                         static_cast<point_value_type>(2.5),
//                                         static_cast<point_value_type>(3.5));
//
//   pcr::core::Point<point_value_type> p2(static_cast<point_value_type>(60.5),
//                                         static_cast<point_value_type>(80.5),
//                                         static_cast<point_value_type>(0.5));
//
//   pcr::core::Point<point_value_type> p3(static_cast<point_value_type>(0.0),
//                                         static_cast<point_value_type>(0.0),
//                                         static_cast<point_value_type>(0.0));
//
//   SECTION("Add empty iterator") {
//     pcr::core::PointCloud pc{};
//     std::array<pcr::core::Point<point_value_type>, 0> points{};
//     pc.(points.begin(), points.end());
//
//     CHECK(pc.size() == 0);
//     CHECK(pc.is_empty());
//   }
//
//   SECTION("Add single element iterator") {
//     pcr::core::PointCloud pc{};
//     std::array<pcr::core::Point<point_value_type>, 1> points{p1};
//     pc.add(points.begin(), points.end());
//
//     CHECK(pc.size() == 1);
//     CHECK_FALSE(pc.is_empty());
//   }
//
//   SECTION("Add multiple element iterator") {
//     pcr::core::PointCloud pc{};
//     std::array<pcr::core::Point<point_value_type>, 3> points{p1, p2, p3};
//     pc.add(points.begin(), points.end());
//
//     CHECK(pc.size() == 3);
//     CHECK_FALSE(pc.is_empty());
//   }
// }

TEST_CASE("is_empty() works correctly", "[is_empty]") {

  SECTION("Returns true when empty") {
    pcr::core::PointCloud pc{};

    CHECK(pc.empty());
  }

  SECTION("Returns false when 1 element") {
    pcr::core::PointCloud pc{};
    pcr::core::Point<point_value_type> p1(static_cast<point_value_type>(1.5),
                                          static_cast<point_value_type>(2.5),
                                          static_cast<point_value_type>(3.5));
    pc.push_back(p1);

    CHECK_FALSE(pc.empty());
  }

  SECTION("Returns false when multiple elements") {
    pcr::core::PointCloud pc{};
    pcr::core::Point<point_value_type> p1(static_cast<point_value_type>(1.5),
                                          static_cast<point_value_type>(2.5),
                                          static_cast<point_value_type>(3.5));
    pcr::core::Point<point_value_type> p2(static_cast<point_value_type>(60.5),
                                          static_cast<point_value_type>(80.5),
                                          static_cast<point_value_type>(0.5));
    pc.push_back(p1);
    pc.push_back(p2);

    CHECK_FALSE(pc.empty());
  }
}

TEST_CASE("begin(), works correctly", "[begin]") {
  pcr::core::Point<point_value_type> p1(static_cast<point_value_type>(1.5),
                                        static_cast<point_value_type>(2.5),
                                        static_cast<point_value_type>(3.5));

  pcr::core::Point<point_value_type> p2(static_cast<point_value_type>(60.5),
                                        static_cast<point_value_type>(80.5),
                                        static_cast<point_value_type>(0.5));

  SECTION("Empty point cloud") {
    pcr::core::PointCloud pc{};

    CHECK(pc.begin() == pc.end());
  }

  SECTION("Non-empty point cloud") {
    pcr::core::PointCloud pc{};
    pc.push_back(p1);
    pc.push_back(p2);

    CHECK(pc.begin() != pc.end());
    CHECK(are_points_equal(*pc.begin(), p1));
  }
}

TEST_CASE("end(), works correctly", "[end]") {
  pcr::core::Point<point_value_type> p1(static_cast<point_value_type>(1.5),
                                        static_cast<point_value_type>(2.5),
                                        static_cast<point_value_type>(3.5));

  pcr::core::Point<point_value_type> p2(static_cast<point_value_type>(60.5),
                                        static_cast<point_value_type>(80.5),
                                        static_cast<point_value_type>(0.5));

  SECTION("Empty point cloud") {
    pcr::core::PointCloud pc{};

    CHECK(pc.begin() == pc.end());
  }

  SECTION("Non-empty point cloud") {
    pcr::core::PointCloud pc{};
    pc.push_back(p1);
    pc.push_back(p2);

    auto it = pc.begin();
    ++it;
    ++it;

    CHECK(it == pc.end());
  }
}


TEST_CASE("Subscripting operator", "[subscript_op]") {
  pcr::core::Point<point_value_type> p1(static_cast<point_value_type>(1.5),
                                        static_cast<point_value_type>(2.5),
                                        static_cast<point_value_type>(3.5));
  pcr::core::Point<point_value_type> p2(static_cast<point_value_type>(60.5),
                                        static_cast<point_value_type>(80.5),
                                        static_cast<point_value_type>(0.5));
  pcr::core::Point<point_value_type> p3(static_cast<point_value_type>(0.0),
                                        static_cast<point_value_type>(0.0),
                                        static_cast<point_value_type>(0.0));

  SECTION("Access first element") {
    pcr::core::PointCloud pc{};
    pc.push_back(p1);
    pc.push_back(p2);
    pc.push_back(p3);

    CHECK(are_points_equal(pc[0], p1));
  }

  SECTION("Access middle element") {
    pcr::core::PointCloud pc{};
    pc.push_back(p1);
    pc.push_back(p2);
    pc.push_back(p3);

    CHECK(are_points_equal(pc[1], p2));
  }

  SECTION("Access last element") {
    pcr::core::PointCloud pc{};
    pc.push_back(p1);
    pc.push_back(p2);
    pc.push_back(p3);

    CHECK(are_points_equal(pc[2], p3));
  }

  SECTION("Const access works correctly") {
    std::array<pcr::core::Point<point_value_type>, 3> points{p1, p2, p3};
    const pcr::core::PointCloud pc(points.begin(), points.end());

    CHECK(are_points_equal(pc[0], p1));
    CHECK(are_points_equal(pc[1], p2));
    CHECK(are_points_equal(pc[2], p3));
  }

  SECTION("Modify element through subscript operator") {
    pcr::core::PointCloud pc{};
    pc.push_back(p1);

    pcr::core::Point<point_value_type> p_new(
        static_cast<point_value_type>(99.9),
        static_cast<point_value_type>(88.8),
        static_cast<point_value_type>(77.7));
    pc[0] = p_new;

    CHECK(are_points_equal(pc[0], p_new));
    CHECK_FALSE(are_points_equal(pc[1], p1));
  }
}