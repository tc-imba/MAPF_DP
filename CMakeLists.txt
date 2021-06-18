cmake_minimum_required(VERSION 3.15)
project(MAPF_DP)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(MAPF_DP main.cpp Solver.cpp CBSSolver.cpp Graph.cpp)