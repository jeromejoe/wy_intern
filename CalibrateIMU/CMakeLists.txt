cmake_minimum_required(VERSION 3.17)
project(imu_calib)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
include_directories("/usr/include/eigen3"
		     ${EIGEN_INCLUDE_DIR}
                    ${CERES_INCLUDE_DIRS})

add_library(calculation calculation.cpp)
add_executable(imu_calib main.cpp)
target_link_libraries(imu_calib calculation ${CERES_LIBRARIES})


