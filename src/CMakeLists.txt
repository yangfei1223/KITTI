add_library(kitti_road
        road.cpp)
target_link_libraries(kitti_road
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES})

add_library(kitti_odometry
        odometry.cpp)
target_link_libraries(kitti_odometry
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES})
