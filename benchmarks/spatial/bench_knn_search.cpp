#include <pcr/io/ply.hpp>
#include <pcr/spatial/kd_tree.hpp>
#include <string>
#include <iostream>
#include <filesystem>
#include <benchmark/benchmark.h>

#define STRING(x) #x
#define XSTRING(x) STRING(x)

std::string bunny_045 {"/bunny/data/bun045.ply"};


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
    for (auto point: pc) {
      std::vector<pcr::point_idx> indices;
      std::vector<pcr::dist_t> distances;

      kd.knn_search(point, k, indices, distances);
    }
  }
}

BENCHMARK(knn_search_benchmark)->Arg(10)-> Arg(50)->Unit(benchmark::kMillisecond);
BENCHMARK_MAIN();