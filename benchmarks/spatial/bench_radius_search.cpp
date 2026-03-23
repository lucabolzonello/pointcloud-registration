#include <pcr/io/ply.hpp>
#include <pcr/spatial/kd_tree.hpp>
#include <string>
#include <iostream>
#include <filesystem>
#include <benchmark/benchmark.h>

#define STRING(x) #x
#define XSTRING(x) STRING(x)

std::string bunny_045{"/bunny/data/bun045.ply"};

static const std::vector<float> float_args = {0.003f, 0.01f};

static void radius_search_benchmark(benchmark::State& s)
{
    std::string filepath = XSTRING(DATASETS_PATH) + bunny_045;

    pcr::core::PointCloud pc = pcr::io::ply::read_file(filepath);

    pcr::spatial::KdTree kd;
    kd.build_index(&pc);
    pcr::dist_t radius = float_args[s.range(0)];

    std::vector<pcr::point_idx> indices;
    std::vector<pcr::dist_t> distances;
    for (auto _ : s)
    {
        for (auto point : pc)
        {
            std::vector<pcr::point_idx> indices;
            std::vector<pcr::dist_t> distances;

            kd.radius_search(point, radius, indices, distances);
        }
    }
}

BENCHMARK(radius_search_benchmark) -> Arg(

0
)
->
Arg (

1
)
->
Unit (benchmark::kMillisecond);
BENCHMARK_MAIN();