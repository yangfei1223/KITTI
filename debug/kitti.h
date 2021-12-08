//
// Created by yangfei on 18-1-21.
//

#ifndef KITTI_KITTI_H
#define KITTI_KITTI_H



namespace KITTI
{
    // odometry dataset
    class Odometry
    {
    private:
        char m_str_dir[128];     // path to datasets
        char m_str_sequence[128];     // sequence
        const int m_max_points_num = 1000000;
        int m_frameID;
        Mat m_P0;    // project cam 0
        Mat m_P1;     // project cam 1
        Mat m_P2;     // project cam 2
        Mat m_P3;     // project cam 1
        Mat m_Tr_velo_to_cam;     // transform lidar to cam 0
        Mat m_image_0;    // image 0 gray
        Mat m_image_1;    // image 1 gray
        Mat m_image_2;    // image 2 color
        Mat m_image_3;    // image 3 color
        int m_points_num;      // lidar points num
        float *m_pLidar;      // lidar data
        Mat m_Grid;      // lidar grid
        vector<Mat> m_pose;
        vector<float> m_time_stamp;
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
        Mat getImg(int n)
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
                    return Mat();
            }
        }
        void getLidar(float *pData,int *num)
        {
            pData=m_pLidar;
            *num=m_points_num;
        }
        void Debug();
        Mat getHeightImg(int n);
        Mat getDepthImg(int n);
        Mat getIntrinsicsOfCameraN(int n);
        Mat getTransformFromLidar2CameraN(int n);
        void Initialize(const char *dir,const char *seq);
        void Evaluate(const char *res_file, const char *gt_file);
        void DoNext(int frameID);

    };


    // road dataset
    class Road
    {
    private:
        bool m_isTrain;		// true, if train set
        char m_str_dir[128];     // path to datasets
        char m_str_type[123];		// data type um,umm or uu
        const int m_max_points_num = 1000000;
        int m_frameID;

        float m_mean[3]={82.33479532,86.90471276,84.28602359};      // BGR mean value of trainset
        // calib data
        Mat m_P0;    // project cam 0
        Mat m_P1;     // project cam 1
        Mat m_P2;     // project cam 2
        Mat m_P3;     // project cam 1
        Mat m_R0_rect;
        Mat m_Tr_velo_to_cam;     // transform lidar to cam 0
        Mat m_Tr_imu_to_velo;	  // transform imu to lidar
        Mat m_Tr_cam_to_road;	  // transform cam to road
        int m_width, m_height;
        Mat m_image_l;    // image 2 color
        Mat m_image_r;    // image 3 color
        Mat m_image_gt;
        int m_points_num;      // lidar points num
        float *m_pLidar;      // lidar data
        Mat m_Mask;      // lidar grid
        Mat m_Height;	 // height image
        Mat m_Lidar2Img;
        vector<Point3f> m_points;		// point set in camera view
        vector<vector<float>> m_xyzrgb;
        vector<int> m_labels;		// correspand label
        void readCalibrationFile();
        void readImages();
        void readLidarData();
    public:
        // constructor
        Road();
        // destructor
        ~Road();
        int getFrameID()
        {
            return m_frameID;
        }
        Mat getImageL()
        {
            return m_image_l.clone();
        }
        Mat getImageR()
        {
            return m_image_r.clone();
        }
        Mat getGroundTruth()
        {
            return m_image_gt.clone();
        }
        void getLidar(float *pData, int *num)
        {
            pData = m_pLidar;
            *num = m_points_num;
        }

        void Debug();
        Mat getHeightImg();
        Mat getDepthImg();
        Mat getLidarGrid();
        Mat getLidarMask()
        {
            return m_Mask.clone();
        }
        void projectLidar2Image();
        void preparePointCloud();
        void savePointsCloudAsTxt();
        void saveXYZRGBCloudAsTxt();
        void saveLidarResAsImg();
        void Initialize(const char *dir,const char *type, bool isTrain);
        void Evaluate(const char *res_file, const char *gt_file);
        void DoNext(int frameID);
    };

}
#endif //KITTI_KITTI_H
