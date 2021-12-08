//
// Created by yangfei on 18-9-29.
//

#ifndef KITTI_ODOMETRY_H
#define KITTI_ODOMETRY_H

#include "global.h"

// odometry dataset
class Odometry
{
private:
    char m_str_dir[128];     // path to datasets
    char m_str_sequence[128];     // sequence
    const int m_max_points_num = 1000000;
    int m_frameID;
    cv::Mat m_P0;    // project cam 0
    cv::Mat m_P1;     // project cam 1
    cv::Mat m_P2;     // project cam 2
    cv::Mat m_P3;     // project cam 1
    cv::Mat m_Tr_velo_to_cam;     // transform lidar to cam 0
    cv::Mat m_image_0;    // image 0 gray
    cv::Mat m_image_1;    // image 1 gray
    cv::Mat m_image_2;    // image 2 color
    cv::Mat m_image_3;    // image 3 color
    int m_points_num;      // lidar points num
    float *m_pLidar;      // lidar data
    cv::Mat m_Grid;      // lidar grid
    std::vector<cv::Mat> m_pose;
    std::vector<float> m_time_stamp;
    void readCalibrationFile();
    void readImages();
    void readLidarData();
public:
    // constructor
    Odometry();
    // destructor
    ~Odometry();
    int getFrameID()
    {
        return m_frameID;
    }
    cv::Mat getImg(int n)
    {
        switch(n)
        {
            case 0:
                return m_image_0.clone();
            case 1:
                return m_image_1.clone();
            case 2:
                return m_image_2.clone();
            case 3:
                return m_image_3.clone();
            default:
                return cv::Mat();
        }
    }
    void getLidar(float *pData,int *num)
    {
        pData=m_pLidar;
        *num=m_points_num;
    }
    void Debug();
    cv::Mat getHeightImg(int n);
    cv::Mat getDepthImg(int n);
    cv::Mat getIntrinsicsOfCameraN(int n);
    cv::Mat getTransformFromLidar2CameraN(int n);
    void Initialize(const char *dir,const char *seq);
    void Evaluate(const char *res_file, const char *gt_file);
    void DoNext(int frameID);

};
#endif //KITTI_ODOMETRY_H
