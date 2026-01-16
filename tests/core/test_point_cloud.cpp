#define CATCH_CONFIG_MAIN
#include <array>
#include <catch2/catch_all.hpp>

#include "pcr/core/point.hpp"
#include "pcr/core/point_cloud.hpp"

using point_value_type = float;

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
    CHECK(pc.is_empty());
  }

  SECTION("Construct from single element Iterator") {
    std::array<pcr::core::Point<point_value_type>, 1> points{p1};
    pcr::core::PointCloud pc(points.begin(), points.end());

    CHECK(pc.size() == 1);
    CHECK_FALSE(pc.is_empty());
  }

  SECTION("Construct form multi element Iterator") {
    std::array<pcr::core::Point<point_value_type>, 3> points{p1, p2, p3};
    pcr::core::PointCloud pc(points.begin(), points.end());

    CHECK(pc.size() == 3);
    CHECK_FALSE(pc.is_empty());
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
    pc.add(p1);

    CHECK(pc.size() == 1);
    CHECK_FALSE(pc.is_empty());
  }

  SECTION("Add multiple points") {
    pcr::core::PointCloud pc{};
    pc.add(p1);
    pc.add(p2);
    pc.add(p3);

    CHECK(pc.size() == 3);
    CHECK_FALSE(pc.is_empty());
  }
}

TEST_CASE("Add point from iterator", "[add]") {
  pcr::core::Point<point_value_type> p1(static_cast<point_value_type>(1.5),
                                        static_cast<point_value_type>(2.5),
                                        static_cast<point_value_type>(3.5));

  pcr::core::Point<point_value_type> p2(static_cast<point_value_type>(60.5),
                                        static_cast<point_value_type>(80.5),
                                        static_cast<point_value_type>(0.5));

  pcr::core::Point<point_value_type> p3(static_cast<point_value_type>(0.0),
                                        static_cast<point_value_type>(0.0),
                                        static_cast<point_value_type>(0.0));

  SECTION("Add empty iterator") {
    pcr::core::PointCloud pc{};
    std::array<pcr::core::Point<point_value_type>, 0> points{};
    pc.add(points.begin(), points.end());

    CHECK(pc.size() == 0);
    CHECK(pc.is_empty());
  }

  SECTION("Add single element iterator") {
    pcr::core::PointCloud pc{};
    std::array<pcr::core::Point<point_value_type>, 1> points{p1};
    pc.add(points.begin(), points.end());

    CHECK(pc.size() == 1);
    CHECK_FALSE(pc.is_empty());
  }

  SECTION("Add multiple element iterator") {
    pcr::core::PointCloud pc{};
    std::array<pcr::core::Point<point_value_type>, 3> points{p1, p2, p3};
    pc.add(points.begin(), points.end());

    CHECK(pc.size() == 3);
    CHECK_FALSE(pc.is_empty());
  }
}

TEST_CASE("is_empty() works correctly", "[is_empty]") {

  SECTION("Returns true when empty") {
    pcr::core::PointCloud pc{};

    CHECK(pc.is_empty());
  }

  SECTION("Returns false when 1 element") {
    pcr::core::PointCloud pc{};
    pcr::core::Point<point_value_type> p1(static_cast<point_value_type>(1.5),
                                          static_cast<point_value_type>(2.5),
                                          static_cast<point_value_type>(3.5));
    pc.add(p1);

    CHECK_FALSE(pc.is_empty());
  }

  SECTION("Returns false when multiple elements") {
    pcr::core::PointCloud pc{};
    pcr::core::Point<point_value_type> p1(static_cast<point_value_type>(1.5),
                                          static_cast<point_value_type>(2.5),
                                          static_cast<point_value_type>(3.5));
    pcr::core::Point<point_value_type> p2(static_cast<point_value_type>(60.5),
                                          static_cast<point_value_type>(80.5),
                                          static_cast<point_value_type>(0.5));
    pc.add(p1);
    pc.add(p2);

    CHECK_FALSE(pc.is_empty());
  }
}

TEST_CASE("begin(), works correctly") {
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
    pc.add(p1);
    pc.add(p2);

    CHECK(pc.begin() != pc.end());
    CHECK(*pc.begin() == p1);
  }
}

TEST_CASE("end(), works correctly") {
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
    pc.add(p1);
    pc.add(p2);

    auto it = pc.begin();
    ++it;
    ++it;

    CHECK(it == pc.end());
  }
}
