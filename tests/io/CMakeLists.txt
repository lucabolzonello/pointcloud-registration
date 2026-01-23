find_package(Catch2 3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(test_ply test_ply.cpp ${CMAKE_SOURCE_DIR}/src/io/ply.cpp ${CMAKE_SOURCE_DIR}/src/core/point_cloud.cpp)

set_project_warnings(test_ply)

target_link_libraries(test_ply
        PRIVATE
        Catch2::Catch2WithMain
)

add_test(ctest_ply test_ply)
