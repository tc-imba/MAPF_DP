cmake_minimum_required(VERSION 3.15)
project(MAPF_DP)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
#add_definitions("-D DEBUG_CYCLE")

set(SOURCE_FILES
        src/Solver.cpp
        src/CBSSolver.cpp
        src/Graph.cpp
        src/DefaultSimulator.cpp
        src/DefaultSimulator.h
        src/OnlineSimulator.cpp
        src/OnlineSimulator.h
)

add_executable(MAPF_DP src/main.cpp ${SOURCE_FILES})

add_executable(MAPF_Map_Generator src/generator/MapGenerator.cpp src/Graph.cpp)
target_link_libraries(MAPF_Map_Generator ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

add_executable(MAPF_Task_Generator src/generator/TaskGenerator.cpp src/Graph.cpp)
target_link_libraries(MAPF_Task_Generator ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})


add_subdirectory(EECBS)

