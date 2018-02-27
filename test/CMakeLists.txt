cmake_minimum_required (VERSION 2.6)
project (test_geotree)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wno-long-long -pedantic")

find_package(GTest REQUIRED)
find_package (Eigen3 REQUIRED)

#FILE(GLOB SRCFILES  ../src/*.cc)
FILE(GLOB SRCFILES test.cc mesh.cc ../src/MeshFactory.cc ../src/Intersection.cc ../src/Geotree.cc ../src/Mesh.cc ../src/Face.cc ../src/Segment.cc ../src/Calc.cc ../src/Log.cc ../src/Validate.cc ../src/Point.cc)
FILE(GLOB SRCFILES_TEST intersection.cc)

add_executable(${PROJECT_NAME} ${SRCFILES} ${SRCFILES_TEST})

include_directories("../src")
include_directories("/home/oscu/git/fast-poly2tri")
include_directories("/home/oscu/src/eigen-eigen-5a0156e40feb")

target_link_libraries(${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES})
