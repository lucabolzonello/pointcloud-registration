
find_package(Catch2 3 REQUIRED)

add_executable(test_point test_point.cpp)

target_link_libraries(test_point
        PRIVATE
        Catch2::Catch2WithMain
)

include_directories(${CMAKE_SOURCE_DIR}/include)
add_test(ctest_point test_point)
