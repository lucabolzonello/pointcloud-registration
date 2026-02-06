#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include <filesystem>

#include "pcr/core/point_cloud.hpp"
#include "pcr/io/ply.hpp"

TEST_CASE("PLY write/read round-trip preserves points", "[ply]") {
  pcr::core::PointCloud cloud;

  cloud.add({static_cast<pcr::coord_t>(1.0), static_cast<pcr::coord_t>(2.0),
             static_cast<pcr::coord_t>(3.0)});

  cloud.add({static_cast<pcr::coord_t>(-1.5), static_cast<pcr::coord_t>(0.0),
             static_cast<pcr::coord_t>(4.2)});

  cloud.add({static_cast<pcr::coord_t>(10.0), static_cast<pcr::coord_t>(20.0),
             static_cast<pcr::coord_t>(30.0)});

  cloud.add({static_cast<pcr::coord_t>(-10.5), static_cast<pcr::coord_t>(65.2),
             static_cast<pcr::coord_t>(32.1)});

  cloud.add({static_cast<pcr::coord_t>(-10.3), static_cast<pcr::coord_t>(65.2),
             static_cast<pcr::coord_t>(32.1)});

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
