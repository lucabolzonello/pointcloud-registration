#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>

#include "pcr/core/point.hpp"

using test_types = std::tuple<std::size_t, float, double>;

TEMPLATE_LIST_TEST_CASE(
    "Point is default constructable, and uses zero initialization",
    "[constructor]", test_types) {
  pcr::core::Point<TestType> p1;
  CHECK(p1.x == TestType{});
  CHECK(p1.y == TestType{});
  CHECK(p1.z == TestType{});
}

TEMPLATE_LIST_TEST_CASE("Constructor with arguments initializes coordinates",
                        "[constructor]", test_types) {

  auto x_value = static_cast<TestType>(1);
  auto y_value = static_cast<TestType>(2);
  auto z_value = static_cast<TestType>(3);

  pcr::core::Point<TestType> p1(x_value, y_value, z_value);
  CHECK(p1.x == x_value);
  CHECK(p1.y == y_value);
  CHECK(p1.z == z_value);
}

TEMPLATE_LIST_TEST_CASE("Point coordinates can be modified", "[member access]",
                        test_types) {
  auto x_value = static_cast<TestType>(1);
  auto y_value = static_cast<TestType>(2);
  auto z_value = static_cast<TestType>(3);

  pcr::core::Point<TestType> p1(x_value, y_value, z_value);

  SECTION("Set X Coordinate") {
    auto new_x_value = static_cast<TestType>(99);
    p1.x = new_x_value;
    CHECK(p1.x == new_x_value);
  }

  SECTION("Set Y Coordinate") {
    auto new_y_value = static_cast<TestType>(10);
    p1.y = new_y_value;
    CHECK(p1.y == new_y_value);
  }

  SECTION("Set Z Coordinate") {
    auto new_z_value = static_cast<TestType>(42);
    p1.z = new_z_value;
    CHECK(p1.z == new_z_value);
  }
}
