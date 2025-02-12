cmake_minimum_required(VERSION 3.18)
project(pathfinding)

option(PATHFINDING_BUILD_EXAMPLES "Build examples" ON)

set(CMAKE_CXX_STANDARD 20)


include(GNUInstallDirs)

add_library(pathfinding INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/include/pathfinding/Coordinate.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/pathfinding/Heuristic.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/pathfinding/Path.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/pathfinding/PathFinder.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/pathfinding/TileBasedMap.hpp"

        "${CMAKE_CURRENT_SOURCE_DIR}/include/pathfinding/algorithm/AStar.hpp"

        "${CMAKE_CURRENT_SOURCE_DIR}/include/pathfinding/heuristics/ClosestHeuristic.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/pathfinding/heuristics/ClosestSquaredHeuristic.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/pathfinding/heuristics/ManhattanHeuristic.hpp"
)
target_include_directories(pathfinding INTERFACE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
)


if (PATHFINDING_BUILD_EXAMPLES)

    include(FetchContent)
    set(THREEPP_BUILD_TESTS OFF)
    set(THREEPP_BUILD_EXAMPLES OFF)
    FetchContent_Declare(
            threepp
            GIT_REPOSITORY https://github.com/markaren/threepp.git
            GIT_TAG dbe0e63df67f0cd18ae0c77933136ed5fd3f51a8
    )
    FetchContent_MakeAvailable(threepp)

    add_subdirectory(examples)
endif ()


install(TARGETS pathfinding EXPORT pathfinding-targets)
install(DIRECTORY "include/" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(EXPORT pathfinding-targets NAMESPACE pathfinding:: DESTINATION "${CMAKE_INSTALL_DATADIR}/pathfinding")
