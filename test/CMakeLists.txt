cmake_minimum_required (VERSION 2.6)
project (test_geotree)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wextra -Werror -pedantic")

find_package(GTest REQUIRED)
find_package (Eigen3 REQUIRED)

FILE(GLOB SRCFILES test.cc intersectionpoints.cc mesh.cc cc face.cc segment.cc meshdivide.cc facedivide.cc point.cc paths.cc
../src/Segment.cc ../src/Mesh.cc ../src/MeshFactory.cc ../src/IntersectionPoints.cc ../src/Face.cc ../src/Point.cc
../src/MeshDivide.cc ../src/FaceDivide.cc ../src/Log.cc ../src/Point.cc ../src/Paths.cc)

add_executable(${PROJECT_NAME} ${SRCFILES})

include_directories("../src")
#include_directories("/home/oscu/git/fast-poly2tri")
include_directories("/home/oscu/local/include/eigen3")

target_link_libraries(${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES})
