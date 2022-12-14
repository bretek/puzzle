cmake_minimum_required(VERSION 3.15)
project(puzzle)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(puzzle src/main.cpp src/generator.cpp src/bspline.cpp)
target_link_libraries(puzzle ${OpenCV_LIBS})