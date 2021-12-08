//
// Created by yangfei on 18-9-29.
//

#ifndef KITTI_ROAD_H
#define KITTI_ROAD_H

#include "global.h"

/// road dataset
class Road
{
private:
    bool m_isTrain;		// true, if train set
    char m_str_dir[128];     // path to datasets
    char m_str_type[123];		// data type um,umm or uu
    int m_frameID;

    /// calib data
    cv::Mat m_P0;    // project cam 0
    cv::Mat m_P1;     // project cam 1
    cv::Mat m_P2;     // project cam 2
    cv::Mat m_P3;     // project cam 3
    cv::Mat m_R0_rect;
    cv::Mat m_Tr_velo_to_cam;     // transform lidar to cam 0
    cv::Mat m_Tr_imu_to_velo;	  // transform imu to lidar
    cv::Mat m_Tr_cam_to_road;	  // transform cam to road
    /// image data
    int m_width, m_height;
    cv::Mat m_image_l;    // image 2 color
    cv::Mat m_image_r;    // image 3 color
    cv::Mat m_image_gt;   // ground truth
    cv::Mat m_sparse_x;   // sparse x -- depth
    cv::Mat m_sparse_y;   // sparse y -- no meaningful
    cv::Mat m_sparse_z;   // sparse z -- height
    /// velodyne data
    int m_nPoints;        // lidar points num
    float *m_pLidar;      // lidar data
    pcl::PointCloud<pcl::PointXYZI> m_pointCloud;       // orgin cloud
    pcl::PointCloud<pcl::Normal> m_normal1;      // compute normal and cuvrature, scale 1
    pcl::PointCloud<pcl::Normal> m_normal2;      // compute normal and cuvrature, scale 2
    pcl::PointCloud<pcl::Normal> m_normal3;      // compute normal and cuvrature, scale 3
    /// fusion data
    cv::Mat m_Indices;      // 保存点云的索引
    cv::Mat m_Height;	 // height image

    /// data process functions
    void readCalibrationFile();
    void readImages();
    void readLidarData();
    void computeNormal(pcl::PointCloud<pcl::Normal> &normal, int k);
    void projectLidar2Image();

public:
    /// constructor
    Road();
    /// destructor
    ~Road();
    /// get functions
    int getImageL(cv::Mat &dst)
    {
        dst=m_image_l;
        return m_frameID;
    }
    int getImageR(cv::Mat &dst)
    {
        dst=m_image_r;
        return m_frameID;
    }
    int getGroundTruth(cv::Mat &dst)
    {
        dst=m_image_gt;
        return m_frameID;
    }
    int getHeightImg(cv::Mat &dst)
    {
        dst=m_Height;
        return m_frameID;
    }
    int getLidar(pcl::PointCloud<pcl::PointXYZINormal> &cloud)
    {
        pcl::concatenateFields(m_pointCloud, m_normal2, cloud);
        return m_frameID;
    }

    void PointCloudSlice();

    /// save functions
    void saveXYZINormalAsPCDFile();
    void saveXYZRGBNormalAsPCDFile();
    void saveUVXYZINormalRGBLabelAsTxtFile();
    void saveUVXYZMultiNormalRGBLabelAsTxtFile();
    void saveGTImage();
    void saveSparseCloudImages();

    /// Initialize
    void Initialize(const char *dir,const char *type, bool isTrain);

    /// DoNext
    void DoNext(int frameID);

    /// Debug
    void Debug();
};

#endif //KITTI_ROAD_H
