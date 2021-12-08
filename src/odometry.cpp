//
// Created by yangfei on 18-9-29.
//

#include "odometry.h"

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
//        std::cout<<m_P0<<std::endl<<m_P1<<std::endl<<m_P2<<std::endl<<m_P3<<std::endl;

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
//        std::cout<<m_Tr_velo_to_cam<<std::endl;
    fclose(fp);
}

void Odometry::readImages()
{
    char filename[128];
    sprintf(filename,"/%s/dataset/sequences/%s/image_0/%06d.png",m_str_dir,m_str_sequence,m_frameID);
    m_image_0=cv::imread(filename,CV_LOAD_IMAGE_UNCHANGED);
    sprintf(filename,"/%s/dataset/sequences/%s/image_1/%06d.png",m_str_dir,m_str_sequence,m_frameID);
    m_image_1=cv::imread(filename,CV_LOAD_IMAGE_UNCHANGED);
    sprintf(filename,"/%s/dataset/sequences/%s/image_2/%06d.png",m_str_dir,m_str_sequence,m_frameID);
    m_image_2=cv::imread(filename,CV_LOAD_IMAGE_UNCHANGED);
    sprintf(filename,"/%s/dataset/sequences/%s/image_3/%06d.png",m_str_dir,m_str_sequence,m_frameID);
    m_image_3=cv::imread(filename,CV_LOAD_IMAGE_UNCHANGED);
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

cv::Mat Odometry::getIntrinsicsOfCameraN(int n)
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
            return cv::Mat();
    }
}

cv::Mat Odometry::getTransformFromLidar2CameraN(int n)
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
            return cv::Mat();
    }
    cv::Mat t;
    t = (cv::Mat_<float>(3,1)<<(t1-t3*cx)/fx,(t2-t3*cy)/fy,t3);
//        std::cout<<"t: \n"<<t<<std::endl;
    cv::Mat T(3,4,CV_32F,cv::Scalar(0));
    m_Tr_velo_to_cam.colRange(0,3).copyTo(T.colRange(0,3));
    T.col(3)=m_Tr_velo_to_cam.col(3)+t;
    return T.clone();
}
void Odometry::Debug()
{
    int i, x, y;
    float *p;
    cv::Mat p_l;    //lidar point
    cv::Mat p_c;    //camera point
    cv::Mat p_p;    //pixel point
    cv::Size size(m_image_2.cols,m_image_2.rows);
    for (i = 0, p = m_pLidar; i < m_points_num; i++, p += 4)
    {
        p_l=(cv::Mat_<float>(3,1) << *p, *(p + 1), *(p + 2));
        if (p_l.at<float>(0,0) < 80 && p_l.at<float>(0,0) > 5)      //select region in camera view
        {
            p_c = m_Tr_velo_to_cam.colRange(0,3)*p_l+m_Tr_velo_to_cam.col(3);     //to camera
            p_p = m_P2.colRange(0,3)*p_c+m_P2.col(3);       //to pixel
            x = cvRound(p_p.at<float>(0,0)/p_p.at<float>(0,2));
            y = cvRound(p_p.at<float>(1,0)/p_p.at<float>(0,2));
            if (x >= 0 && x < size.width && y >= 0 && y < size.height)
            {
                m_image_2.at<cv::Vec3b>(y,x)=cv::Vec3b(0,255,0);
            }
        }
    }
}
cv::Mat Odometry::getDepthImg(int n)
{
    cv::Mat T=getTransformFromLidar2CameraN(n);
    cv::Mat K=getIntrinsicsOfCameraN(n);
//        std::cout<<"T_src: \n"<<m_Tr_velo_to_cam<<std::endl;
//        std::cout<<"T_dst: \n"<<T<<std::endl;
//        std::cout<<"K: \n"<<K<<std::endl;
    int i, x, y;
    float *p;
    cv::Mat p_l;    //lidar point
    cv::Mat p_c;    //camera point
    cv::Mat p_p;    //pixel point
    cv::Size size(m_image_2.cols,m_image_2.rows);
    cv::Mat depth(size, CV_16U, cv::Scalar(0));
    float max_depth=0;
    for (i = 0, p = m_pLidar; i < m_points_num; i++, p += 4)
    {
        p_l=(cv::Mat_<float>(3,1) << *p, *(p + 1), *(p + 2));
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
                m_image_2.at<cv::Vec3b>(y,x)=cv::Vec3b(0,0,255);
            }
        }
    }
    std::cout<<"max depth is "<<max_depth<<std::endl;
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
    std::vector<cv::Mat> VecRes,VecGt;

    while(!feof(fp1))
    {
        float temp=0;
        cv::Mat tr1(3,4,CV_32F,cv::Scalar(0));
        cv::Mat tr2(3,4,CV_32F,cv::Scalar(0));
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
    cv::Mat depth = getDepthImg(2);
    char filename[128];
    std::vector<int> compression_params;
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
