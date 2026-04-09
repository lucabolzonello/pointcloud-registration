find_package(Catch2 3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

# basic kd tree tests
add_executable(test_icp test_icp.cpp ${CMAKE_SOURCE_DIR}/src/spatial/kd_tree.cpp ${CMAKE_SOURCE_DIR}/src/core/point_cloud.cpp ${CMAKE_SOURCE_DIR}/src/registration/icp.cpp)


set_project_warnings(test_icp)

target_link_libraries(test_icp
        PRIVATE
        Catch2::Catch2WithMain
        Eigen3::Eigen
)

add_test(ctest_icp test_icp)

