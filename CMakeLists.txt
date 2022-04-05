cmake_minimum_required(VERSION 3.0.0)
project(pathfinding VERSION 0.1.0)

find_package(Boost)
include_directories(${Boost_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a -O3 -Wall -Wextra")

set(SOURCE_FILES
    graph.cpp
    main.cpp
    reservation_table.cpp
    reverse_resumable_a_star.cpp
    safe_interval_path_planning.cpp
    utils.cpp
    zone_capacity_constraint.cpp)

set(HEADER_FILES
    graph.hpp
    reservation_table.hpp
    reverse_resumable_a_star.hpp
    safe_interval_path_planning.hpp
    structs.hpp
    utils.hpp
    zone_capacity_constraint.hpp)

add_executable(pathfinding ${SOURCE_FILES} ${HEADER_FILES})
