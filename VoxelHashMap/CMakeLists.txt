cmake_minimum_required(VERSION 3.17)
project(VoxelMap)

set(CMAKE_CXX_STANDARD 14)

find_package(Pangolin)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(VoxelMap main.cpp VoxelHashMap.cpp ./Visualizer/BasaltVisualizer.cpp)
target_link_libraries(VoxelMap pangolin ${OpenCV_LIBS})
