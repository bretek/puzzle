cmake_minimum_required(VERSION 3.15)
project(puzzleGenerator)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(GSL REQUIRED)

add_executable(puzzleGenerator src/generator.cpp src/bspline.cpp)
target_link_libraries(puzzleGenerator ${OpenCV_LIBS})
target_link_libraries(puzzleGenerator GSL::gsl)