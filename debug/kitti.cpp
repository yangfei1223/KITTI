//
// Created by yangfei on 18-1-21.
//
#include "kitti.h"
namespace KITTI
{
    Odometry::Odometry()
    {
    }
    Odometry::~Odometry()
    {
        if(m_pLidar)    delete[] m_pLidar;
    }
    void Odometry::Initialize(const char *dir, const char *seq)
    {
        sprintf(m_str_dir,"%s",dir);
        sprintf(m_str_sequence,"%s",seq);
        m_pLidar=new float[m_max_points_num*sizeof(float)];
        m_P0.create(3,4,CV_32F);
        m_P1.create(3,4,CV_32F);
        m_P2.create(3,4,CV_32F);
        m_P3.create(3,4,CV_32F);
        m_Tr_velo_to_cam.create(3,4,CV_32F);
    }
    void Odometry::readCalibrationFile()
    {
        char filename[128];
        sprintf(filename,"%s/dataset/sequences/%s/calib.txt",m_str_dir,m_str_sequence);
        FILE *fp;
        if ((fp = fopen(filename, "r")) == NULL)
        {
            printf("can not open %s !\n",filename);
            return;
        }
        char index[128];
        char str[128];
        float temp;
        fseek(fp,0,SEEK_SET);       //指向文件头
        // P0
        sprintf(index,"P0:");
        while(1)
        {
            fscanf(fp,"%s",str);
            if(!strcmp(str,index))
                break;
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 4; j++)
            {

                fscanf(fp,"%f",&temp);
                m_P0.at<float>(i,j)=temp;
            }
        }
        // P1
        sprintf(index,"P1:");
        while(1)
        {
            fscanf(fp,"%s",str);
            if(!strcmp(str,index))
                break;
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 4; j++)
            {

                fscanf(fp,"%f",&temp);
                m_P1.at<float>(i,j)=temp;
            }
        }
        // P2
        sprintf(index,"P2:");
        while(1)
        {
            fscanf(fp,"%s",str);
            if(!strcmp(str,index))
                break;
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 4; j++)
            {

                fscanf(fp,"%f",&temp);
                m_P2.at<float>(i,j)=temp;
            }
        }
        // P3
        sprintf(index,"P3:");
        while(1)
        {
            fscanf(fp,"%s",str);
            if(!strcmp(str,index))
                break;
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 4; j++)
            {

                fscanf(fp,"%f",&temp);
                m_P3.at<float>(i,j)=temp;
            }
        }
//        cout<<m_P0<<endl<<m_P1<<endl<<m_P2<<endl<<m_P3<<endl;

        // Tr_velo_to_cam
        sprintf(index,"Tr:");
        while(1)
        {
            fscanf(fp,"%s",str);
            if(!strcmp(str,index))
                break;
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                fscanf(fp,"%f",&temp);
                m_Tr_velo_to_cam.at<float>(i,j)=temp;
            }
        }
//        cout<<m_Tr_velo_to_cam<<endl;
        fclose(fp);
    }

    void Odometry::readImages()
    {
        char filename[128];
        sprintf(filename,"/%s/dataset/sequences/%s/image_0/%06d.png",m_str_dir,m_str_sequence,m_frameID);
        m_image_0=imread(filename,CV_LOAD_IMAGE_UNCHANGED);
        sprintf(filename,"/%s/dataset/sequences/%s/image_1/%06d.png",m_str_dir,m_str_sequence,m_frameID);
        m_image_1=imread(filename,CV_LOAD_IMAGE_UNCHANGED);
        sprintf(filename,"/%s/dataset/sequences/%s/image_2/%06d.png",m_str_dir,m_str_sequence,m_frameID);
        m_image_2=imread(filename,CV_LOAD_IMAGE_UNCHANGED);
        sprintf(filename,"/%s/dataset/sequences/%s/image_3/%06d.png",m_str_dir,m_str_sequence,m_frameID);
        m_image_3=imread(filename,CV_LOAD_IMAGE_UNCHANGED);
    }

    void Odometry::readLidarData()
    {
        char filename[128];
        sprintf(filename,"/%s/dataset/sequences/%s/velodyne/%06d.bin",m_str_dir,m_str_sequence,m_frameID);
        memset(m_pLidar,0,sizeof(float)*m_max_points_num);
        m_points_num = m_max_points_num;
        FILE *fp = NULL;
        if ((fp = fopen(filename, "rb")) == NULL)
        {
            printf("can not open file %s !\n",filename);
            return;
        }
        m_points_num = fread(m_pLidar, sizeof(float), m_points_num, fp) / 4;
//        printf("points num is  %d\n", m_points_num);
        fclose(fp);
    }

    Mat Odometry::getIntrinsicsOfCameraN(int n)
    {
        switch(n)
        {
            case 0:
                return m_P0.colRange(0,3).clone();
            case 1:
                return m_P1.colRange(0,3).clone();
            case 2:
                return m_P2.colRange(0,3).clone();
            case 3:
                return m_P3.colRange(0,3).clone();
            default:
                return Mat();
        }
    }

    Mat Odometry::getTransformFromLidar2CameraN(int n)
    {
        float t1,t2,t3,fx,fy,cx,cy;
        switch(n)
        {
            case 0:
                t1 = m_P0.at<float>(0,3);
                t2 = m_P0.at<float>(1,3);
                t3 = m_P0.at<float>(2,3);
                fx = m_P0.at<float>(0,0);
                fy = m_P0.at<float>(0,2);
                cx = m_P0.at<float>(1,1);
                cy = m_P0.at<float>(1,2);
                break;
            case 1:
                t1 = m_P1.at<float>(0,3);
                t2 = m_P1.at<float>(1,3);
                t3 = m_P1.at<float>(2,3);
                fx = m_P1.at<float>(0,0);
                fy = m_P1.at<float>(0,2);
                cx = m_P1.at<float>(1,1);
                cy = m_P1.at<float>(1,2);
                break;
            case 2:
                t1 = m_P2.at<float>(0,3);
                t2 = m_P2.at<float>(1,3);
                t3 = m_P2.at<float>(2,3);
                fx = m_P2.at<float>(0,0);
                fy = m_P2.at<float>(0,2);
                cx = m_P2.at<float>(1,1);
                cy = m_P2.at<float>(1,2);
                break;
            case 3:
                t1 = m_P3.at<float>(0,3);
                t2 = m_P3.at<float>(1,3);
                t3 = m_P3.at<float>(2,3);
                fx = m_P3.at<float>(0,0);
                fy = m_P3.at<float>(0,2);
                cx = m_P3.at<float>(1,1);
                cy = m_P3.at<float>(1,2);
                break;
            default:
                return Mat();
        }
        Mat t;
        t = (Mat_<float>(3,1)<<(t1-t3*cx)/fx,(t2-t3*cy)/fy,t3);
//        cout<<"t: \n"<<t<<endl;
        Mat T(3,4,CV_32F,Scalar(0));
        m_Tr_velo_to_cam.colRange(0,3).copyTo(T.colRange(0,3));
        T.col(3)=m_Tr_velo_to_cam.col(3)+t;
        return T.clone();
    }
    void Odometry::Debug()
    {
        int i, x, y;
        float *p;
        Mat p_l;    //lidar point
        Mat p_c;    //camera point
        Mat p_p;    //pixel point
        Size size(m_image_2.cols,m_image_2.rows);
        for (i = 0, p = m_pLidar; i < m_points_num; i++, p += 4)
        {
            p_l=(Mat_<float>(3,1) << *p, *(p + 1), *(p + 2));
            if (p_l.at<float>(0,0) < 80 && p_l.at<float>(0,0) > 5)      //select region in camera view
            {
                p_c = m_Tr_velo_to_cam.colRange(0,3)*p_l+m_Tr_velo_to_cam.col(3);     //to camera
                p_p = m_P2.colRange(0,3)*p_c+m_P2.col(3);       //to pixel
                x = cvRound(p_p.at<float>(0,0)/p_p.at<float>(0,2));
                y = cvRound(p_p.at<float>(1,0)/p_p.at<float>(0,2));
                if (x >= 0 && x < size.width && y >= 0 && y < size.height)
                {
                    m_image_2.at<Vec3b>(y,x)=Vec3b(0,255,0);
                }
            }
        }
    }
    Mat Odometry::getDepthImg(int n)
    {
        Mat T=getTransformFromLidar2CameraN(n);
        Mat K=getIntrinsicsOfCameraN(n);
//        cout<<"T_src: \n"<<m_Tr_velo_to_cam<<endl;
//        cout<<"T_dst: \n"<<T<<endl;
//        cout<<"K: \n"<<K<<endl;
        int i, x, y;
        float *p;
        Mat p_l;    //lidar point
        Mat p_c;    //camera point
        Mat p_p;    //pixel point
        Size size(m_image_2.cols,m_image_2.rows);
        Mat depth(size, CV_16U, cv::Scalar(0));
        float max_depth=0;
        for (i = 0, p = m_pLidar; i < m_points_num; i++, p += 4)
        {
            p_l=(Mat_<float>(3,1) << *p, *(p + 1), *(p + 2));
            if (p_l.at<float>(0,0) < 80 && p_l.at<float>(0,0) > 5)      //select region in camera view
            {
                p_c = T.colRange(0,3)*p_l+T.col(3);     //to camera
                p_p = K*p_c;       //to pixel
                x = cvRound(p_p.at<float>(0,0)/p_p.at<float>(0,2));
                y = cvRound(p_p.at<float>(1,0)/p_p.at<float>(0,2));
                if (x >= 0 && x < size.width && y >= 0 && y < size.height)
                {
                    float d = p_c.at<float>(2,0);
                    if(d>max_depth)
                        max_depth=d;
                    depth.at<unsigned short>(y,x)=cvRound(d*800.0);     // scale factor 800
                    m_image_2.at<Vec3b>(y,x)=Vec3b(0,0,255);
                }
            }
        }
        cout<<"max depth is "<<max_depth<<endl;
        return depth;
    }

    void Odometry::Evaluate(const char *res_file, const char *gt_file)
    {
        cv::Mat traj = cv::Mat::zeros(1000, 1000, CV_8UC3);
        char text[100];
        cv::Point text_org(10, 50);
        double t[3]={};
        FILE *fp1= fopen(res_file,"rt");
        FILE *fp2= fopen(gt_file,"rt");
        if(!fp1||!fp2)
        {
            std::cout<<"open file error !\n";
            exit(-1);
        }
        vector<Mat> VecRes,VecGt;

        while(!feof(fp1))
        {
            float temp=0;
            Mat tr1(3,4,CV_32F,Scalar(0));
            Mat tr2(3,4,CV_32F,Scalar(0));
            for(int i=0;i<3;i++)
            {
                for(int j=0;j<4;j++)
                {
                    fscanf(fp1,"%f",&temp);
                    tr1.at<float>(i,j)=temp;
                    fscanf(fp2,"%f",&temp);
                    tr2.at<float>(i,j)=temp;
                }
            }
            VecRes.push_back(tr1);
            VecGt.push_back(tr2);
        }
        //process frame by frame
        for (size_t i = 0; i < VecRes.size(); i++)
        {
            int x1,y1,z1,x2,y2,z2;
            x1=cvRound(VecRes[i].at<float>(0, 3));
            y1=cvRound(VecRes[i].at<float>(1, 3));
            z1=cvRound(VecRes[i].at<float>(2, 3));
            x2=cvRound(VecGt[i].at<float>(0, 3));
            y2=cvRound(VecGt[i].at<float>(1, 3));
            z2=cvRound(VecGt[i].at<float>(2, 3));

            cv::circle(traj, cv::Point(x1+300, z1+300), 1, CV_RGB(255, 0, 0), 2);
            cv::circle(traj, cv::Point(x2+300, z2+300), 1, CV_RGB(0, 255, 0), 2);

            cv::rectangle(traj, cv::Point(10, 30), cv::Point(800, 60), CV_RGB(0, 0, 0), CV_FILLED);
            sprintf(text, "Frame: %d   Coordinates: x = %02fm y = %02fm z = %02fm", i, x1, y1, z1);
            cv::putText(traj, text, text_org, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar::all(255), 1, 8);
            cv::imshow("Trajectory", traj);
            cv::waitKey(1);
        }
        imwrite("../res_rgbd.png",traj);
    }


    void Odometry::DoNext(int frameID)
    {
        m_frameID=frameID;
        readCalibrationFile();
        readImages();
        readLidarData();
//        Debug();
        Mat depth = getDepthImg(2);
        char filename[128];
        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); //PNG格式图片的压缩级别
        compression_params.push_back(0);  //这里设置保存的图像质量级别
        sprintf(filename,"../depth_02/%06d.png",m_frameID);
        imwrite(filename,depth,compression_params);
//        depth.convertTo(depth,CV_8U,3/5000.0);
//        imshow("depth",depth);
//        imshow("image_2",m_image_2);
//        waitKey(0);
//        destroyAllWindows();

    }



    Road::Road()
    {
    }
    Road::~Road()
    {
        if (m_pLidar)    delete[] m_pLidar;
    }
    void Road::Initialize(const char *dir, const char *type, bool isTrain)
    {
        m_isTrain = isTrain;
        if(m_isTrain)
            sprintf(m_str_dir, "%s/training", dir);
        else
            sprintf(m_str_dir, "%s/testing", dir);
        sprintf(m_str_type, "%s", type);
        m_pLidar = new float[m_max_points_num * sizeof(float)];
        m_P0.create(3, 4, CV_32F);
        m_P1.create(3, 4, CV_32F);
        m_P2.create(3, 4, CV_32F);
        m_P3.create(3, 4, CV_32F);
        m_R0_rect.create(3, 3, CV_32F);
        m_Tr_velo_to_cam.create(3, 4, CV_32F);
        m_Tr_imu_to_velo.create(3, 4, CV_32F);
        m_Tr_cam_to_road.create(3, 4, CV_32F);
        m_points.reserve(20000);
        if (m_isTrain)
            m_labels.reserve(20000);
    }
    void Road::readCalibrationFile()
    {
        char filename[128];
        sprintf(filename, "%s/calib/%s_%06d.txt", m_str_dir,m_str_type,m_frameID);
        FILE *fp;
        if ((fp = fopen(filename, "r")) == NULL)
        {
            printf("can not open %s !\n", filename);
            return;
        }
        char index[128];
        char str[128];
        float temp;
        fseek(fp, 0, SEEK_SET);       //指向文件头
        // P0
        sprintf(index, "P0:");
        while (1)
        {
            fscanf(fp, "%s", str);
            if (!strcmp(str, index))
                break;
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 4; j++)
            {

                fscanf(fp, "%f", &temp);
                m_P0.at<float>(i, j) = temp;
            }
        }
        // P1
        sprintf(index, "P1:");
        while (1)
        {
            fscanf(fp, "%s", str);
            if (!strcmp(str, index))
                break;
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 4; j++)
            {

                fscanf(fp, "%f", &temp);
                m_P1.at<float>(i, j) = temp;
            }
        }
        // P2
        sprintf(index, "P2:");
        while (1)
        {
            fscanf(fp, "%s", str);
            if (!strcmp(str, index))
                break;
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 4; j++)
            {

                fscanf(fp, "%f", &temp);
                m_P2.at<float>(i, j) = temp;
            }
        }
        // P3
        sprintf(index, "P3:");
        while (1)
        {
            fscanf(fp, "%s", str);
            if (!strcmp(str, index))
                break;
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 4; j++)
            {

                fscanf(fp, "%f", &temp);
                m_P3.at<float>(i, j) = temp;
            }
        }
        // R0_rect
        sprintf(index, "R0_rect:");
        while (1)
        {
            fscanf(fp, "%s", str);
            if (!strcmp(str, index))
                break;
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                fscanf(fp, "%f", &temp);
                m_R0_rect.at<float>(i, j) = temp;
            }
        }
        // Tr_velo_to_cam
        sprintf(index, "Tr_velo_to_cam:");
        while (1)
        {
            fscanf(fp, "%s", str);
            if (!strcmp(str, index))
                break;
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                fscanf(fp, "%f", &temp);
                m_Tr_velo_to_cam.at<float>(i, j) = temp;
            }
        }
        // Tr_imu_to_velo
        sprintf(index, "Tr_imu_to_velo:");
        while (1)
        {
            fscanf(fp, "%s", str);
            if (!strcmp(str, index))
                break;
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                fscanf(fp, "%f", &temp);
                m_Tr_imu_to_velo.at<float>(i, j) = temp;
            }
        }
        // Tr_cam_to_road
        sprintf(index, "Tr_cam_to_road:");
        while (1)
        {
            fscanf(fp, "%s", str);
            if (!strcmp(str, index))
                break;
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                fscanf(fp, "%f", &temp);
                m_Tr_cam_to_road.at<float>(i, j) = temp;
            }
        }
        fclose(fp);
    }

    void Road::readImages()
    {
        char filename[128];
        sprintf(filename, "%s/image_2/%s_%06d.png", m_str_dir, m_str_type, m_frameID);
        m_image_l = imread(filename, CV_LOAD_IMAGE_UNCHANGED);
        m_width = m_image_l.cols;
        m_height = m_image_l.rows;
        sprintf(filename, "%s/image_3/%s_%06d.png", m_str_dir, m_str_type, m_frameID);
        m_image_r = imread(filename, CV_LOAD_IMAGE_UNCHANGED);
        if(m_isTrain)
        {
            sprintf(filename, "%s/gt_image_2/%s_road_%06d.png", m_str_dir, m_str_type, m_frameID);
            m_image_gt= imread(filename, CV_LOAD_IMAGE_UNCHANGED);
        }
    }

    void Road::readLidarData()
    {
        char filename[128];
        sprintf(filename, "%s/velodyne/%s_%06d.bin", m_str_dir, m_str_type, m_frameID);
        memset(m_pLidar, 0, sizeof(float)*m_max_points_num);
        m_points_num = m_max_points_num;
        FILE *fp = NULL;
        if ((fp = fopen(filename, "rb")) == NULL)
        {
            printf("can not open file %s !\n", filename);
            return;
        }
        m_points_num = fread(m_pLidar, sizeof(float), m_points_num, fp) / 4;
        //        printf("points num is  %d\n", m_points_num);
        fclose(fp);
    }

    void Road::projectLidar2Image()
    {
        int i, x, y;
        int count = 0;
        float *p;
        Mat p_l;    //lidar point
        Mat p_c;    //camera point
        Mat p_p;    //pixel point
        Mat xyz(m_height, m_width, CV_32FC3, Scalar::all(0));
        Mat mask(m_height, m_width, CV_8U, Scalar(0));
        Mat height(m_height, m_width, CV_8U, Scalar(0));
        for (i = 0, p = m_pLidar; i < m_points_num; i++, p += 4)
        {
            p_l = (Mat_<float>(3, 1) << *p, *(p + 1), *(p + 2));
            if (p_l.at<float>(0, 0) < 80 && p_l.at<float>(0, 0) > 5)      //select region in camera view
            {
                p_c = m_Tr_velo_to_cam.colRange(0, 3)*p_l + m_Tr_velo_to_cam.col(3);     //to camera
                p_p = m_P2.colRange(0,3)*m_R0_rect*p_c+m_P2.col(3);       //to pixel

                x = cvRound(p_p.at<float>(0, 0) / p_p.at<float>(2, 0));
                y = cvRound(p_p.at<float>(1, 0) / p_p.at<float>(2, 0));
                if (x >= 0 && x < m_width && y >= 0 && y < m_height)
                {
                    Vec3f vec(p_l.at<float>(0, 0), p_l.at<float>(1, 0), p_l.at<float>(2, 0));
                    xyz.at<Vec3f>(y, x) = vec;
                    mask.at<uchar>(y, x) = 255;
                    height.at<uchar>(y, x) = min(max(0,int((p_l.at<float>(2, 0) + 2.0) * 255 / 4.0)),255);
//                    m_image_l.at<Vec3b>(y, x) = Vec3b(0,0,255);
                    count++;
                }
            }
        }

        xyz.copyTo(m_Lidar2Img);
        mask.copyTo(m_Mask);
        height.copyTo(m_Height);
#if 0
        cout << "valid points num is " << count << endl;
		count = 0;
		for (int i = 0; i < m_height; i++)
		{
			for (int j = 0; j < m_width; j++)
			{
				if (m_Mask.at<uchar>(i, j) > 0)
					count++;
			}
		}
		cout << "real valid points num is " << count << endl;
#endif
    }
    void Road::preparePointCloud()
    {
        m_points.clear();
        m_xyzrgb.clear();
        if (m_isTrain)
            m_labels.clear();
        for (int i=0;i<m_height;i++)
        {
            for(int j=0;j<m_width;j++)
            {
                if (m_Mask.at<unsigned char>(i, j) == 0)
                    continue;
                float b = m_image_l.at<Vec3b>(i, j)[0]-m_mean[0];       // sub mean
                float g = m_image_l.at<Vec3b>(i, j)[1]-m_mean[0];
                float r = m_image_l.at<Vec3b>(i, j)[2]-m_mean[0];
                if(m_isTrain)		// for train
                {
                    int bb = m_image_gt.at<Vec3b>(i, j)[0];
                    int gg = m_image_gt.at<Vec3b>(i, j)[1];
                    int rr = m_image_gt.at<Vec3b>(i, j)[2];
                    if (rr > 0)		// valid area
                    {
                        Point3f pt(m_Lidar2Img.at<Vec3f>(i, j)[0], m_Lidar2Img.at<Vec3f>(i, j)[1], m_Lidar2Img.at<Vec3f>(i, j)[2]);
                        m_points.push_back(pt);
                        float tmp[8]={i, j, m_Lidar2Img.at<Vec3f>(i, j)[0], m_Lidar2Img.at<Vec3f>(i, j)[1], m_Lidar2Img.at<Vec3f>(i, j)[2], r,g,b};
                        vector<float> vec(begin(tmp),end(tmp));
                        m_xyzrgb.push_back(vec);
                        if (bb > 0)		// positive
                            m_labels.push_back(1);
                        else
                            m_labels.push_back(0);
                    }
                }
                else     // for test
                {
                    Point3f pt(m_Lidar2Img.at<Vec3f>(i, j)[0], m_Lidar2Img.at<Vec3f>(i, j)[1], m_Lidar2Img.at<Vec3f>(i, j)[2]);
                    m_points.push_back(pt);
                    float tmp[8]={i, j, m_Lidar2Img.at<Vec3f>(i, j)[0], m_Lidar2Img.at<Vec3f>(i, j)[1], m_Lidar2Img.at<Vec3f>(i, j)[2], r,g,b};
                    vector<float> vec(begin(tmp),end(tmp));
                    m_xyzrgb.push_back(vec);
                }
            }
        }
    }

    void Road::savePointsCloudAsTxt()
    {

        char filename[128];
        sprintf(filename, "%s/points/%s_%06d.txt", m_str_dir, m_str_type, m_frameID);
        FILE *fp = fopen(filename, "wt");
        if (fp == NULL)
            printf("can not open file %s ！\n", filename);

        cout << m_points.size()<<" "<<m_labels.size()<< endl;
        for (size_t i=0;i<m_points.size();i++)
        {
            if (m_isTrain)
                fprintf(fp, "%f\t%f\t%f\t%d\n", m_points[i].x, m_points[i].y, m_points[i].z, m_labels[i]);
            else
                fprintf(fp, "%f\t%f\t%f\n", m_points[i].x, m_points[i].y, m_points[i].z);
        }
        if (fp)	fclose(fp);
    }

    void Road::saveXYZRGBCloudAsTxt()
    {
        char filename[128];
        sprintf(filename, "%s/uvxyzrgb/%s_%06d.txt", m_str_dir, m_str_type, m_frameID);
        FILE *fp = fopen(filename, "wt");
        if (fp == NULL)
            printf("can not open file %s ！\n", filename);

        cout << m_xyzrgb.size()<<" "<<m_labels.size()<< endl;
        for (size_t i=0;i<m_xyzrgb.size();i++)
        {
            if (m_isTrain)
                fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n", m_xyzrgb[i][0], m_xyzrgb[i][1], m_xyzrgb[i][2],m_xyzrgb[i][3], m_xyzrgb[i][4], m_xyzrgb[i][5], m_xyzrgb[i][6], m_xyzrgb[i][7], m_labels[i]);
            else
                fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", m_xyzrgb[i][0], m_xyzrgb[i][1], m_xyzrgb[i][2],m_xyzrgb[i][3], m_xyzrgb[i][4], m_xyzrgb[i][5], m_xyzrgb[i][6], m_xyzrgb[i][7]);
        }
        if (fp)	fclose(fp);
    }

    void Road::saveLidarResAsImg()
    {
        Mat res(m_height, m_width, CV_8UC3, Scalar::all(0));
        Mat prob(m_height, m_width, CV_32F, Scalar(0));
        char filename[128];
        sprintf(filename, "%s/result/%s_%06d.txt", m_str_dir, m_str_type, m_frameID);
        FILE *fp = fopen(filename, "rt");
        for (int i=0;i<m_height;i++)
        {
            for (int j=0;j<m_width;j++)
            {
                if (m_Mask.at<uchar>(i, j) == 0)
                {
//                    prob.at<float>(i, j) = 0.5;
                    continue;
                }
                float logit = 0;
                fscanf(fp, "%f", &logit);
                //cout << logit << endl;
                Vec3b color;
                if (logit > 0.5)
                    color = Vec3b(0, 255, 0);
                else
                    color = Vec3b(0, 0, 255);
                res.at<Vec3b>(i, j) = color;
                prob.at<float>(i, j) = logit;
            }
        }
        fclose(fp);
//        sprintf(filename, "%s/test_images_rg/%s_%06d.png", m_str_dir, m_str_type, m_frameID);
//        imwrite(filename, res);
        sprintf(filename, "%s/test_images_sparse/%s_%06d.png", m_str_dir, m_str_type, m_frameID);
        prob.convertTo(prob, CV_8U, 255, 0);
        imwrite(filename, prob);

    }

    Mat Road::getLidarGrid()
    {
        const int grid_height = 400;
        const int grid_width = 800;
        const float grid_res_x = 0.2;
        const float grid_res_y = 0.1;
        Mat Grid(grid_height, grid_width, CV_8U, Scalar(0));
        for(int i=0;i<m_Lidar2Img.rows;i++)
        {
            for(int j=0;j<m_Lidar2Img.cols;j++)
            {
                if(m_Mask.at<uchar>(i,j)==0)
                    continue;
                float x = m_Lidar2Img.at<Vec3f>(i, j)[0];
                float y = m_Lidar2Img.at<Vec3f>(i, j)[1];
                int grid_y = grid_height-x / grid_res_x;
                int grid_x = -y / grid_res_y + grid_width/2;
                if (grid_x > 0 && grid_x < grid_width&&grid_y>0 && grid_y < grid_height)
                    Grid.at<uchar>(grid_y, grid_x) = 255;
            }
        }
#if 0
        namedWindow("grid");
		imshow("grid", Grid);
		waitKey(0);
		destroyWindow("grid");
#endif
        return Grid.clone();
    }

    void Road::DoNext(int frameID)
    {
        m_frameID = frameID;
        readCalibrationFile();
        readImages();
        readLidarData();
        projectLidar2Image();
        preparePointCloud();
        saveXYZRGBCloudAsTxt();
    }
}
