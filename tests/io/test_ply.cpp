#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include <filesystem>

#include "pcr/core/point_cloud.hpp"
#include "pcr/io/ply.hpp"

TEST_CASE("PLY write/read round-trip preserves points", "[ply]") {
  pcr::core::PointCloud cloud;

  cloud.add({1.0f, 2.0f, 3.0f});
  cloud.add({-1.5f, 0.0f, 4.2f});
  cloud.add({10.0f, 20.0f, 30.0f});
  cloud.add({-10.5f, 65.2f, 32.1f});
  cloud.add({-10.3f, 65.2f, 32.1f});
  const auto path = std::filesystem::temp_directory_path() / "pcr_test_cloud";

  SECTION("Binary PLY") {
    pcr::io::ply::write_file(path.string(), cloud, /*binary=*/true);

    pcr::core::PointCloud loaded =
        pcr::io::ply::read_file(path.string() + ".ply");

    REQUIRE(loaded.size() == cloud.size());

    auto it1 = cloud.begin();
    auto it2 = loaded.begin();

    for (; it1 != cloud.end(); ++it1, ++it2) {
      CHECK_THAT(it1->x, Catch::Matchers::WithinRel(it2->x));
      CHECK_THAT(it1->y, Catch::Matchers::WithinRel(it2->y));
      CHECK_THAT(it1->z, Catch::Matchers::WithinRel(it2->z));
    }
  }

  SECTION("ASCII PLY") {
    pcr::io::ply::write_file(path.string(), cloud, /*binary=*/false);

    pcr::core::PointCloud loaded =
        pcr::io::ply::read_file(path.string() + ".ply");

    REQUIRE(loaded.size() == cloud.size());
    auto it1 = cloud.begin();
    auto it2 = loaded.begin();

    for (; it1 != cloud.end(); ++it1, ++it2) {
      CHECK_THAT(it1->x, Catch::Matchers::WithinRel(it2->x));
      CHECK_THAT(it1->y, Catch::Matchers::WithinRel(it2->y));
      CHECK_THAT(it1->z, Catch::Matchers::WithinRel(it2->z));
    }
  }

  std::filesystem::remove(path.string() + ".ply");
}
