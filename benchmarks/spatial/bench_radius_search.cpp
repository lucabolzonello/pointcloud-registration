#include <pcr/io/ply.hpp>
#include <pcr/spatial/kd_tree.hpp>
#include <string>
#include <iostream>
#include <filesystem>
#include <cstddef>
#include <benchmark/benchmark.h>

#define STRING(x) #x
#define XSTRING(x) STRING(x)

std::string bunny_045{"/bunny/data/bun045.ply"};

static const std::vector<float> float_args = {0.003f, 0.01f};

static void radius_search_benchmark(benchmark::State &s) {
  std::string filepath = XSTRING(DATASETS_PATH) + bunny_045;

  pcr::core::PointCloud pc = pcr::io::ply::read_file(filepath);

  pcr::spatial::KdTree kd;
  kd.build_index(&pc);
  pcr::dist_t radius = float_args[s.range(0)];

  std::vector<pcr::point_idx> indices;
  std::vector<pcr::dist_t> distances;
  for (auto _ : s) {
    for (auto point : pc) {
      std::vector<pcr::point_idx> indices;
      std::vector<pcr::dist_t> distances;

      kd.radius_search(point, radius, indices, distances);
    }
  }
}


// Brute force radius search for verification
void brute_force_radius(const pcr::core::PointCloud &cloud,
                        const pcr::core::Point<pcr::dist_t> &query,
                        pcr::dist_t radius,
                        std::vector<pcr::point_idx> &indices,
                        std::vector<pcr::dist_t> &distances) {
  indices.clear();
  distances.clear();

  pcr::dist_t radius_sq = radius * radius;
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto &point = cloud[i];
    pcr::dist_t dx = point.x - query.x;
    pcr::dist_t dy = point.y - query.y;
    pcr::dist_t dz = point.z - query.z;
    pcr::dist_t dist_sq = dx * dx + dy * dy + dz * dz;

    if (dist_sq <= radius_sq) {
      distances.push_back(dist_sq);
      indices.push_back(i);
    }
  }
}

static void radius_brute_search_benchmark(benchmark::State &s) {
  std::string filepath = XSTRING(DATASETS_PATH) + bunny_045;

  pcr::core::PointCloud pc = pcr::io::ply::read_file(filepath);

  pcr::dist_t radius = float_args[s.range(0)];

  std::vector<pcr::point_idx> indices;
  std::vector<pcr::dist_t> distances;
  for (auto _ : s) {
    for (auto point : pc) {
      std::vector<pcr::point_idx> indices;
      std::vector<pcr::dist_t> distances;

      brute_force_radius(pc, point, radius, indices, distances);
    }
  }

}


BENCHMARK(radius_search_benchmark)->Arg(0)->Arg(1)->Unit(
    benchmark::kMillisecond);

BENCHMARK(radius_brute_search_benchmark)->Arg(0)->Arg(1)->Unit(
    benchmark::kMillisecond);

BENCHMARK_MAIN();