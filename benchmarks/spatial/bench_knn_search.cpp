#include <pcr/io/ply.hpp>
#include <pcr/spatial/kd_tree.hpp>
#include <string>
#include <iostream>
#include <filesystem>
#include <benchmark/benchmark.h>
#include <cstddef>

#define STRING(x) #x
#define XSTRING(x) STRING(x)

std::string bunny_045{"/bunny/data/bun045.ply"};


static void knn_search_benchmark(benchmark::State &s) {

  std::string filepath = XSTRING(DATASETS_PATH) + bunny_045;

  pcr::core::PointCloud pc = pcr::io::ply::read_file(filepath);

  std::cout << "Number of points: " << pc.size() << std::endl;

  pcr::spatial::KdTree kd;
  kd.build_index(&pc);
  pcr::point_idx k = s.range(0);

  std::vector<pcr::point_idx> indices;
  std::vector<pcr::dist_t> distances;
  for (auto _ : s) {
    for (auto point : pc) {
      std::vector<pcr::point_idx> indices;
      std::vector<pcr::dist_t> distances;

      kd.knn_search(point, k, indices, distances);
    }
  }
}

// Brute force KNN search for verification
void brute_force_knn(const pcr::core::PointCloud &cloud,
                     const pcr::core::Point<pcr::dist_t> &query,
                     size_t k, std::vector<pcr::point_idx> &indices,
                     std::vector<pcr::dist_t> &distances) {
  indices.clear();
  distances.clear();

  std::vector<std::pair<pcr::dist_t, pcr::point_idx>> results;
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto &point = cloud[i];
    pcr::dist_t dx = point.x() - query.x();
    pcr::dist_t dy = point.y() - query.y();
    pcr::dist_t dz = point.z() - query.z();
    pcr::dist_t dist_sq = dx * dx + dy * dy + dz * dz;
    results.emplace_back(dist_sq, i);
  }

  const size_t num_results = std::min(k, results.size());
  std::nth_element(results.begin(), results.begin() + num_results,
                   results.end());

  for (size_t i = 0; i < num_results; ++i) {
    distances.push_back(results[i].first);
    indices.push_back(results[i].second);
  }
}


static void knn_brute_search_benchmark(benchmark::State &s) {
  std::string filepath = XSTRING(DATASETS_PATH) + bunny_045;
  pcr::core::PointCloud pc = pcr::io::ply::read_file(filepath);
  std::cout << "Number of points: " << pc.size() << std::endl;

  pcr::point_idx k = s.range(0);

  std::vector<pcr::point_idx> indices;
  std::vector<pcr::dist_t> distances;
  for (auto _ : s) {
    std::vector<pcr::point_idx> indices;
    std::vector<pcr::dist_t> distances;
    for (auto _ : s) {
      for (auto point : pc) {
        std::vector<pcr::point_idx> indices;
        std::vector<pcr::dist_t> distances;

        brute_force_knn(pc, point, k, indices, distances);

      }
    }
  }
}

BENCHMARK(knn_search_benchmark)->Arg(10)->Arg(50)->
                                 Unit(benchmark::kMillisecond);

BENCHMARK(knn_brute_search_benchmark)->Arg(10)->Arg(50)->
                                       Unit(benchmark::kMillisecond);
BENCHMARK_MAIN();