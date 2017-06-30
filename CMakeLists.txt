cmake_minimum_required (VERSION 2.6)
project (geotree)

find_package (Eigen3 REQUIRED)

FILE(GLOB SRCFILES src/*.cc)

add_library(${PROJECT_NAME} STATIC ${SRCFILES})

include_directories("src")
include_directories("/usr/include/eigen3")

target_include_directories (${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
