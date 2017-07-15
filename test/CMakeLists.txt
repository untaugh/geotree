cmake_minimum_required (VERSION 2.6)
project (test_geotree)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic")

find_package(GTest REQUIRED)
find_package (Eigen3 REQUIRED)

FILE(GLOB SRCFILES *.cc ../src/*.cc)

add_executable(${PROJECT_NAME} ${SRCFILES})

include_directories("../src")
#include_directories("/usr/include/eigen3")
include_directories("/home/oscu/src/eigen-eigen-5a0156e40feb")

target_link_libraries(${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES})
