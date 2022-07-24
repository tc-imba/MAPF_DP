cmake_minimum_required(VERSION 3.15)
project(MAPF_DP)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost)
include_directories(${Boost_INCLUDE_DIRS})
#add_definitions("-D DEBUG_CYCLE")

add_executable(MAPF_DP src/main.cpp src/Solver.cpp src/CBSSolver.cpp src/Graph.cpp src/DefaultSimulator.cpp src/DefaultSimulator.h src/OnlineSimulator.cpp src/OnlineSimulator.h)