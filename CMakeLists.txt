cmake_minimum_required(VERSION 3.14)
project(pathfinding VERSION 0.1.0)

find_package(Boost)
include_directories(${Boost_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra")

file(GLOB LIB_FILES CONFIGURE_DEPENDS src/*.cpp src/*.hpp)

add_library(pathfinding ${LIB_FILES})

file(GLOB TEST_FILES CONFIGURE_DEPENDS test/*.cpp test/*.hpp)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/9e712372214d75bb30ec2847a44bf124d48096f3.zip
)

FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(pathfinding_test ${TEST_FILES})

target_link_libraries(
  pathfinding_test
  pathfinding
  gtest_main
  gmock_main
)

include(GoogleTest)
gtest_discover_tests(pathfinding_test)