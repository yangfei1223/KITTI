cmake_minimum_required(VERSION 3.8)
project(KITTI)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
        include
        ${OPENCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS})

add_subdirectory(src)
add_executable(KITTI main.cpp)
target_link_libraries(KITTI kitti_road kitti_odometry)