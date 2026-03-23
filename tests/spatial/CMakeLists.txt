find_package(Catch2 3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

# basic kd tree tests
add_executable(test_kd_tree test_kd_tree.cpp ${CMAKE_SOURCE_DIR}/src/spatial/kd_tree.cpp ${CMAKE_SOURCE_DIR}/src/core/point_cloud.cpp)

set_project_warnings(test_kd_tree)

target_link_libraries(test_kd_tree
        PRIVATE
        Catch2::Catch2WithMain
)

add_test(ctest_kd_tree test_kd_tree)


# brute force kd tree tests
add_executable(test_kd_tree_brute test_kd_tree_brute.cpp ${CMAKE_SOURCE_DIR}/src/spatial/kd_tree.cpp ${CMAKE_SOURCE_DIR}/src/core/point_cloud.cpp ${CMAKE_SOURCE_DIR}/src/io/ply.cpp)


set_project_warnings(test_kd_tree_brute)

target_link_libraries(test_kd_tree_brute
        PRIVATE
        Catch2::Catch2WithMain
)

add_test(ctest_kd_tree_brute test_kd_tree_brute) # Not sure if I should be officially registering as a test or not (because it is really slow)
