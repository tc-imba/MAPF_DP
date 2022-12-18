cmake_minimum_required(VERSION 3.15)
project(MAPF_DP)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories(${Boost_INCLUDE_DIRS})
#add_definitions("-D DEBUG_CYCLE")

include_directories("EECBS/inc")
file(GLOB SOURCE_FILES "EECBS/src/*.cpp")
list(REMOVE_ITEM SOURCE_FILES "EECBS/src/driver.cpp")

add_executable(EECBS ${SOURCE_FILES} EECBS/src/driver.cpp)
target_link_libraries(EECBS ${Boost_LIBRARIES})
add_library(EECBS_Static ${SOURCE_FILES} src/utils/eecbs.cpp)
target_link_libraries(EECBS_Static ${Boost_LIBRARIES})

set(SOURCE_FILES
        src/Solver.cpp
        src/CBSSolver.cpp
        src/EECBSSolver.cpp
        src/Graph.cpp
        src/DefaultSimulator.cpp
        src/DefaultSimulator.h
        src/OnlineSimulator.cpp
        src/OnlineSimulator.h
)

add_executable(MAPF_DP src/main.cpp ${SOURCE_FILES})
target_link_libraries(MAPF_DP EECBS_Static)

add_executable(MAPF_Map_Generator src/generator/MapGenerator.cpp src/Graph.cpp)
target_link_libraries(MAPF_Map_Generator ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

add_executable(MAPF_Task_Generator src/generator/TaskGenerator.cpp src/Graph.cpp)
target_link_libraries(MAPF_Task_Generator ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})


#add_subdirectory(EECBS)

