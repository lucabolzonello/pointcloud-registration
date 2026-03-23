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

static void ply_write_binary_benchmark(benchmark::State &s) {
  std::string filepath = XSTRING(DATASETS_PATH) + dataset_paths[s.range(0)];
  const auto path = std::filesystem::temp_directory_path() / "pcr_test_cloud";

  const auto out_filepath = path.string() + dataset_paths[s.range(0)] + ".ply";

  pcr::core::PointCloud pc = pcr::io::ply::read_file(filepath);
  for (auto _ : s) {
    pcr::io::ply::write_file(out_filepath, pc, true);
  }

}

static void ply_write_ascii_benchmark(benchmark::State &s) {
  std::string filepath = XSTRING(DATASETS_PATH) + dataset_paths[s.range(0)];
  const auto path = std::filesystem::temp_directory_path() / "pcr_test_cloud";

  const auto out_filepath = path.string() + dataset_paths[s.range(0)] + ".ply";

  pcr::core::PointCloud pc = pcr::io::ply::read_file(filepath);
  for (auto _ : s) {
    pcr::io::ply::write_file(out_filepath, pc, false);
  }

}

BENCHMARK(ply_write_binary_benchmark)->Arg(0)->Arg(1)->Arg(2)->Unit(
    benchmark::kMillisecond);

BENCHMARK(ply_write_ascii_benchmark)->Arg(0)->Arg(1)->Arg(2)->Unit(
    benchmark::kMillisecond);
BENCHMARK_MAIN();