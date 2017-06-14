cmake_minimum_required (VERSION 2.6)
project (test_geotree)

find_package(GTest REQUIRED)
find_package (Eigen3 REQUIRED)


FILE(GLOB SRCFILES *.cc ../src/*.cc)

add_executable(${PROJECT_NAME} ${SRCFILES})

include_directories("../src")
include_directories("/usr/include/eigen3")

target_link_libraries(${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES})
