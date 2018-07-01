cmake_minimum_required (VERSION 2.6)
project (test_geotree)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wextra -Werror -pedantic")

find_package(GTest REQUIRED)
find_package (Eigen3 REQUIRED)

FILE(GLOB SRCFILES ../src/Segment.cc ../src/Mesh.cc ../src/MeshFactory.cc ../src/IntersectionPoints.cc ../src/Face.cc ../src/Point.cc
../src/MeshDivide.cc ../src/FacesDivide.cc ../src/Log.cc ../src/Point.cc ../src/Paths.cc ../src/Triangulate.cc ../src/Cube.cc ../src/EdgePoint.cc ../src/Face2D.cc)

FILE(GLOB TESTSRCFILES test.cc intersectionpoints.cc mesh.cc cc face.cc segment.cc meshdivide.cc facesdivide.cc point.cc paths.cc triangulate.cc face2d.cc loop2d.cc)

add_executable(${PROJECT_NAME} ${SRCFILES} ${TESTSRCFILES})

include_directories("../src")
include_directories("/home/oscu/git/earcut.hpp/include/mapbox")
include_directories("/home/oscu/local/include/eigen3")

target_link_libraries(${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES})
