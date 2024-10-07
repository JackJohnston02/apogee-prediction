cmake_minimum_required(VERSION 3.10)

project(Filter-Comparison)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -O3")

find_package(Eigen3 REQUIRED)
find_package(Matplot++ REQUIRED)

file(GLOB src
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

include_directories("${PROJECT_SOURCE_DIR}/include")

add_executable(${PROJECT_NAME} main.cpp ${src})
target_link_libraries(${PROJECT_NAME} Matplot++::matplot)

configure_file(${CMAKE_SOURCE_DIR}/regulus.csv ${CMAKE_CURRENT_BINARY_DIR}/regulus.csv COPYONLY)