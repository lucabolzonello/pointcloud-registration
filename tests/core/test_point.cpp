#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>

#include "pcr/core/point.hpp"

#include "pcr/prelude.hpp"


void compare_points(const pcr::core::Point<pcr::coord_t> &it1,
                    const pcr::core::Point<pcr::coord_t> &it2) {

  CHECK_THAT(it1.x(), Catch::Matchers::WithinRel(it2.x()));
  CHECK_THAT(it1.y(), Catch::Matchers::WithinRel(it2.y()));
  CHECK_THAT(it1.z(), Catch::Matchers::WithinRel(it2.z()));
}

TEST_CASE("Point is default constructable, and uses zero initialization",
          "[constructor]") {
  pcr::core::Point<pcr::coord_t> p1;
  CHECK(p1.x() == pcr::coord_t{});
  CHECK(p1.y() == pcr::coord_t{});
  CHECK(p1.z() == pcr::coord_t{});
}

TEST_CASE("Constructor with arguments initializes coordinates",
          "[constructor]") {

  auto x_value = static_cast<pcr::coord_t>(1);
  auto y_value = static_cast<pcr::coord_t>(2);
  auto z_value = static_cast<pcr::coord_t>(3);

  pcr::core::Point<pcr::coord_t> p1(x_value, y_value, z_value);
  CHECK(p1.x() == x_value);
  CHECK(p1.y() == y_value);
  CHECK(p1.z() == z_value);
}

TEST_CASE("Point coordinates can be modified", "[member access]") {
  auto x_value = static_cast<pcr::coord_t>(1);
  auto y_value = static_cast<pcr::coord_t>(2);
  auto z_value = static_cast<pcr::coord_t>(3);

  pcr::core::Point<pcr::coord_t> p1(x_value, y_value, z_value);

  SECTION("Set X Coordinate") {
    auto new_x_value = static_cast<pcr::coord_t>(99);
    p1.x() = new_x_value;
    CHECK(p1.x() == new_x_value);
  }

  SECTION("Set Y Coordinate") {
    auto new_y_value = static_cast<pcr::coord_t>(10);
    p1.y() = new_y_value;
    CHECK(p1.y() == new_y_value);
  }

  SECTION("Set Z Coordinate") {
    auto new_z_value = static_cast<pcr::coord_t>(42);
    p1.z() = new_z_value;
    CHECK(p1.z() == new_z_value);
  }
}

TEST_CASE("Operator+= overload adds coordinates from another point",
          "[overload]") {
  auto x_value = static_cast<pcr::coord_t>(1);
  auto y_value = static_cast<pcr::coord_t>(2);
  auto z_value = static_cast<pcr::coord_t>(3);

  pcr::core::Point<pcr::coord_t> p1(x_value, y_value, z_value);
  pcr::core::Point<pcr::coord_t> p2(x_value, y_value, z_value);
  pcr::core::Point<pcr::coord_t> result_point(x_value + x_value,
                                              y_value + y_value,
                                              z_value + z_value);

  p1 += p2;

  compare_points(p1, result_point);
}

TEST_CASE("Operator+ overload returns a new summed point",
          "[overload]") {
  auto x_value = static_cast<pcr::coord_t>(1);
  auto y_value = static_cast<pcr::coord_t>(2);
  auto z_value = static_cast<pcr::coord_t>(3);

  pcr::core::Point<pcr::coord_t> p1(x_value, y_value, z_value);
  pcr::core::Point<pcr::coord_t> p2(x_value, y_value, z_value);

  pcr::core::Point<pcr::coord_t> result_point(x_value + x_value,
                                              y_value + y_value,
                                              z_value + z_value);

  auto p3 = p1 + p2;

  compare_points(p3, result_point);
}

TEST_CASE("Operator/= overload divides coordinates by a scalar",
          "[overload]") {
  auto x_value = static_cast<pcr::coord_t>(2);
  auto y_value = static_cast<pcr::coord_t>(4);
  auto z_value = static_cast<pcr::coord_t>(6);

  pcr::core::Point<pcr::coord_t> p1(x_value, y_value, z_value);
  auto divisor = static_cast<pcr::coord_t>(2);

  pcr::core::Point<pcr::coord_t> result_point(x_value / divisor,
                                              y_value / divisor,
                                              z_value / divisor);
  p1 /= divisor;

  compare_points(p1, result_point);
}

TEST_CASE("Operator/ overload returns a new divided point",
          "[overload]") {
  auto x_value = static_cast<pcr::coord_t>(2);
  auto y_value = static_cast<pcr::coord_t>(4);
  auto z_value = static_cast<pcr::coord_t>(6);

  pcr::core::Point<pcr::coord_t> p1(x_value, y_value, z_value);

  auto divisor = static_cast<pcr::coord_t>(2);
  auto p2 = p1 / divisor;
  pcr::core::Point<pcr::coord_t> result_point(x_value / divisor,
                                              y_value / divisor,
                                              z_value / divisor);

  compare_points(p2, result_point);
}

TEST_CASE("Transform of point by a transformation matrix", "[transform]") {
  pcr::core::Point<pcr::coord_t> p1(1, 2, 3);
  SECTION("Identity transformation") {
    pcr::transform_t identity_transform = pcr::transform_t::Identity();

    pcr::core::Point<pcr::coord_t> result_point = p1;
    p1.transform(identity_transform);

    compare_points(p1, result_point);
  }

  SECTION("Translation") {
    pcr::transform_t translation_transform = pcr::transform_t::Identity();
    translation_transform *= Eigen::Translation<pcr::coord_t, 3>(
        Eigen::Vector3f(2.0f, -2.0f, 3.0f));

    p1.transform(translation_transform);

    pcr::core::Point<pcr::coord_t> result_point(3, 0, 6);
    compare_points(result_point, p1);
  }

  SECTION("Rotation with quaternion") {
    pcr::transform_t rotation_transform = pcr::transform_t::Identity();
    // Rotation
    rotation_transform *= Eigen::Quaternion<
      pcr::coord_t>(0.0, 1.0, 0.0, 0.0);

    p1.transform(rotation_transform);

    pcr::core::Point<pcr::coord_t> result_point(1, -2.0, -3.0);
    compare_points(result_point, p1);
  }
  SECTION("Scaling") {
    pcr::transform_t scaling_transform = pcr::transform_t::Identity();
    scaling_transform *= Eigen::UniformScaling<pcr::coord_t>(2.0);
    p1.transform(scaling_transform);

    pcr::core::Point<pcr::coord_t> result_point(2, 4, 6);
    compare_points(result_point, p1);
  }

  SECTION("Combined rotation and translation") {
    pcr::transform_t combined_transform = pcr::transform_t::Identity();

    combined_transform *= Eigen::Quaternion<pcr::coord_t>(
        0.0, 1.0, 0.0, 0.0);

    combined_transform *= Eigen::Translation<pcr::coord_t, 3>(
        Eigen::Vector3f(2.0f, -2.0f, 3.0f));

    p1.transform(combined_transform);

    pcr::core::Point<pcr::coord_t> result_point(3.0, 0.0, -6.0);
    compare_points(result_point, p1);
  }

}

