include(FetchContent)

function(fetch_catch2)
    FetchContent_Declare(
            catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG v3.11.0
    )
    FetchContent_MakeAvailable(catch2)
endfunction()

function(fetch_google_benchmark)
    FetchContent_Declare(
            benchmark
            GIT_REPOSITORY https://github.com/google/benchmark.git
            GIT_TAG v1.9.5
    )
    set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE) # to get a
    FetchContent_MakeAvailable(benchmark)
endfunction()

function(fetch_eigen)
    FetchContent_Declare(
            eigen
            GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
            GIT_TAG 3.4.0
    )
    FetchContent_MakeAvailable(eigen)
endfunction()