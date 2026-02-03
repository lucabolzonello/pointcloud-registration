find_package(Catch2 3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(test_kd_tree test_kd_tree.cpp ${CMAKE_SOURCE_DIR}/src/spatial/kd_tree.cpp ${CMAKE_SOURCE_DIR}/src/core/point_cloud.cpp)

set_project_warnings(test_kd_tree)

target_link_libraries(test_kd_tree
        PRIVATE
        Catch2::Catch2WithMain
)

add_test(ctest_kd_tree test_kd_tree)