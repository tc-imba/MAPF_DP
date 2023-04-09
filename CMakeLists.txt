cmake_minimum_required(VERSION 3.15)
project(MAPF_DP)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options system filesystem graph)
include_directories(${Boost_INCLUDE_DIRS})
#add_definitions("-D DEBUG_CYCLE")

include_directories("EECBS/inc")
file(GLOB SOURCE_FILES "EECBS/src/*.cpp")
list(REMOVE_ITEM SOURCE_FILES "EECBS/src/main.cpp")

add_executable(EECBS ${SOURCE_FILES} EECBS/src/main.cpp)
target_link_libraries(EECBS ${Boost_LIBRARIES})
add_library(EECBS_Static ${SOURCE_FILES})
target_link_libraries(EECBS_Static ${Boost_LIBRARIES})

include_directories("GeometricTools/GTE")

set(SOURCE_FILES
        src/solver/Solver.cpp
        src/solver/CBSSolver.cpp
        src/solver/EECBSSolver.cpp
        src/Graph.cpp
        src/simulator/DefaultSimulator.cpp
        src/simulator/DefaultSimulator.h
        src/simulator/OnlineSimulator.cpp
        src/simulator/OnlineSimulator.h
        src/simulator/Simulator.cpp
        src/simulator/Simulator.h
        src/solver/IndividualAStarSolver.cpp
        src/solver/IndividualAStarSolver.h
        )

add_executable(MAPF_DP src/main.cpp ${SOURCE_FILES})
IF (WIN32)
    target_link_libraries(MAPF_DP EECBS_Static ws2_32)
ELSE()
    target_link_libraries(MAPF_DP EECBS_Static)
ENDIF()

add_executable(MAPF_Map_Generator src/generator/MapGenerator.cpp src/Graph.cpp)
target_link_libraries(MAPF_Map_Generator ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_GRAPH_LIBRARY})

add_executable(MAPF_Task_Generator src/generator/TaskGenerator.cpp src/Graph.cpp)
target_link_libraries(MAPF_Task_Generator ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_GRAPH_LIBRARY})

add_executable(MAPF_Plan_Validator src/generator/PlanValidator.cpp src/Graph.cpp)
target_link_libraries(MAPF_Plan_Validator ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_GRAPH_LIBRARY})

#add_subdirectory(EECBS)

add_executable(MAPF_test src/test.cpp)


