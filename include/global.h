//
// Created by yangfei on 18-9-29.
//

#ifndef KITTI_GLOBAL_H
#define KITTI_GLOBAL_H

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#define MAX_LIDAR_POINTS_NUM    1000000
#define UM_ROAD_TRAIN 95
#define UMM_ROAD_TRAIN 96
#define UU_ROAD_TRAIN 98
#define UM_ROAD_TEST 96
#define UMM_ROAD_TEST 94
#define UU_ROAD_TEST 100

#endif //KITTI_GLOBAL_H
