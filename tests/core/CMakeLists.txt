find_package(Catch2 3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(test_point test_point.cpp)

set_project_warnings(test_point)

target_link_libraries(test_point
        PRIVATE
        Catch2::Catch2WithMain
)

add_test(ctest_point test_point)

add_executable(test_point_cloud test_point_cloud.cpp ${CMAKE_SOURCE_DIR}/src/core/point_cloud.cpp)

set_project_warnings(test_point_cloud)

target_link_libraries(test_point_cloud
        PRIVATE
        Catch2::Catch2WithMain
)

add_test(ctest_point_cloud test_point_cloud)

