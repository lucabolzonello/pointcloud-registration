#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>

#include "pcr/core/point.hpp"

#include "pcr/prelude.hpp"

TEST_CASE("Point is default constructable, and uses zero initialization",
          "[constructor]") {
  pcr::core::Point<pcr::coord_t> p1;
  CHECK(p1.x == pcr::coord_t{});
  CHECK(p1.y == pcr::coord_t{});
  CHECK(p1.z == pcr::coord_t{});
}

TEST_CASE("Constructor with arguments initializes coordinates",
          "[constructor]") {

  auto x_value = static_cast<pcr::coord_t>(1);
  auto y_value = static_cast<pcr::coord_t>(2);
  auto z_value = static_cast<pcr::coord_t>(3);

  pcr::core::Point<pcr::coord_t> p1(x_value, y_value, z_value);
  CHECK(p1.x == x_value);
  CHECK(p1.y == y_value);
  CHECK(p1.z == z_value);
}

TEST_CASE("Point coordinates can be modified", "[member access]") {
  auto x_value = static_cast<pcr::coord_t>(1);
  auto y_value = static_cast<pcr::coord_t>(2);
  auto z_value = static_cast<pcr::coord_t>(3);

  pcr::core::Point<pcr::coord_t> p1(x_value, y_value, z_value);

  SECTION("Set X Coordinate") {
    auto new_x_value = static_cast<pcr::coord_t>(99);
    p1.x = new_x_value;
    CHECK(p1.x == new_x_value);
  }

  SECTION("Set Y Coordinate") {
    auto new_y_value = static_cast<pcr::coord_t>(10);
    p1.y = new_y_value;
    CHECK(p1.y == new_y_value);
  }

  SECTION("Set Z Coordinate") {
    auto new_z_value = static_cast<pcr::coord_t>(42);
    p1.z = new_z_value;
    CHECK(p1.z == new_z_value);
  }
}

TEST_CASE("Operator== overload works", "[equivalency overload]") {
  auto x_value = static_cast<pcr::coord_t>(1);
  auto y_value = static_cast<pcr::coord_t>(2);
  auto z_value = static_cast<pcr::coord_t>(3);

  SECTION("Points equivalent") {

    pcr::core::Point<pcr::coord_t> p1(x_value, y_value, z_value);
    pcr::core::Point<pcr::coord_t> p2(x_value, y_value, z_value);
  }

  SECTION("Different X value") {

    pcr::core::Point<pcr::coord_t> p1(x_value, y_value, z_value);
    pcr::core::Point<pcr::coord_t> p2(y_value, y_value, z_value);
  }

  SECTION("Different Y value") {

    pcr::core::Point<pcr::coord_t> p1(x_value, y_value, z_value);
    pcr::core::Point<pcr::coord_t> p2(x_value, z_value, z_value);
  }

  SECTION("Different Z value") {

    pcr::core::Point<pcr::coord_t> p1(x_value, y_value, z_value);
    pcr::core::Point<pcr::coord_t> p2(x_value, y_value, x_value);
  }
}