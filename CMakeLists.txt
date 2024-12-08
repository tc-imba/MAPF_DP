cmake_minimum_required(VERSION 3.15)
project(MAPF_DP)
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)

# spdlog
add_subdirectory(spdlog)
include_directories("spdlog/include")

# Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem graph)
include_directories(${Boost_INCLUDE_DIRS})

# EECBS
add_subdirectory(EECBS)

# BTPG
add_subdirectory(BTPG)

# PIBT
add_subdirectory(time-independent-planning)

# GeometricTools
include_directories("GeometricTools/GTE")

# Continuous-CBS
set(CMAKE_CXX_STANDARD 11)
add_subdirectory(Continuous-CBS)

# Switchable-Edge-Search (SES)
file(GLOB SES_SOURCE_FILES
        "Switchable-Edge-Search/src/**/*.cpp"
        "Switchable-Edge-Search/src/**/*.h"
)
add_executable(SES ${SES_SOURCE_FILES} "Switchable-Edge-Search/src/simulate.cpp")
add_library(lib-ses ${SES_SOURCE_FILES})

# MAPF-DP
add_definitions("-D SPDLOG_ACTIVE_LEVEL=1")
set(CMAKE_CXX_STANDARD 20)
file(GLOB SIMULATOR_SOURCE_FILES "src/simulator/*.cpp")
file(GLOB SOLVER_SOURCE_FILES "src/solver/*.cpp")
file(GLOB DEPENDENCY_GRAPH_SOURCE_FILES "src/dependency_graph/*.cpp")
file(GLOB DELAY_SOURCE_FILES "src/delay/*.cpp")
set(SOURCE_FILES
        ${SIMULATOR_SOURCE_FILES}
        ${SOLVER_SOURCE_FILES}
        ${DEPENDENCY_GRAPH_SOURCE_FILES}
        ${DELAY_SOURCE_FILES}
        src/Graph.cpp
        Continuous-CBS/tinyxml2.cpp
)

add_executable(MAPF_DP src/main.cpp ${SOURCE_FILES})
target_link_libraries(MAPF_DP ${Boost_LIBRARIES} lib-time-ind lib-btpg spdlog lib-ses)
IF (WIN32)
    target_link_libraries(MAPF_DP ws2_32)
ENDIF ()

#add_executable(MAPF_Map_Generator src/generator/MapGenerator.cpp src/Graph.cpp Continuous-CBS/tinyxml2.cpp)
#target_link_libraries(MAPF_Map_Generator ${Boost_LIBRARIES})
#
#add_executable(MAPF_Task_Generator src/generator/TaskGenerator.cpp src/Graph.cpp Continuous-CBS/tinyxml2.cpp)
#target_link_libraries(MAPF_Task_Generator ${Boost_LIBRARIES})
#
#add_executable(MAPF_Plan_Validator src/generator/PlanValidator.cpp src/Graph.cpp Continuous-CBS/tinyxml2.cpp)
#target_link_libraries(MAPF_Plan_Validator ${Boost_LIBRARIES})

#add_executable(MAPF_test src/test_graphml.cpp)
#target_link_libraries(MAPF_test ${Boost_LIBRARIES})





