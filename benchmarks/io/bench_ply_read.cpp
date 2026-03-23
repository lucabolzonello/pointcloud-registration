#include <pcr/io/ply.hpp>
#include <string>
#include <iostream>
#include <filesystem>
#include <benchmark/benchmark.h>

#define STRING(x) #x
#define XSTRING(x) STRING(x)

std::vector<std::string> dataset_paths{
    "/bunny/data/bun045.ply", "/bunny/data/bun000.ply", "/bunny/data/bun315.ply"
};

static void ply_read_benchmark(benchmark::State &s) {
  std::string filepath = XSTRING(DATASETS_PATH) + dataset_paths[s.range(0)];

  for (auto _ : s) {
    pcr::core::PointCloud pc = pcr::io::ply::read_file(filepath);
  }

}

BENCHMARK(ply_read_benchmark)->Arg(0)->Arg(1)->Arg(2)->Unit(
    benchmark::kMillisecond);
BENCHMARK_MAIN();