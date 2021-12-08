//
// Created by yangfei on 18-9-29.
//

#include "road.h"


Road::Road()
{
}
Road::~Road()
{
    if (m_pLidar)    delete[] m_pLidar;
}

void Road::readCalibrationFile()
{
    char filename[128];
    sprintf(filename, "%s/calib/%s_%06d.txt", m_str_dir,m_str_type,m_frameID);
    FILE *fp;
    if ((fp = fopen(filename, "r")) == NULL)
    {
        printf("can not open %s !\n", filename);
        exit(-1);
    }
    char index[128];
    char str[128];
    float temp;
    fseek(fp, 0, SEEK_SET);       //指向文件头
    // P0
    sprintf(index, "P0:");
    while (true)
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
    while (true)
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
    while (true)
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
    while (true)
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
    while (true)
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
    while (true)
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
    while (true)
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
    while (true)
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
    int i,j;
    char filename[128];
    sprintf(filename, "%s/image_2/%s_%06d.png", m_str_dir, m_str_type, m_frameID);
    m_image_l = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
    m_width = m_image_l.cols;
    m_height = m_image_l.rows;
    sprintf(filename, "%s/image_3/%s_%06d.png", m_str_dir, m_str_type, m_frameID);
    m_image_r = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
    if(m_isTrain)
    {
        sprintf(filename, "%s/gt_image_2/%s_road_%06d.png", m_str_dir, m_str_type, m_frameID);
        cv::Mat gt_color = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat gt_gray(gt_color.rows,gt_color.cols,CV_8U, cv::Scalar(0));
        for(i=0;i<gt_color.rows;i++)
        {
            for(j=0;j<gt_color.cols;j++)
            {
                int b = gt_color.at<cv::Vec3b>(i, j)[0];
                int g = gt_color.at<cv::Vec3b>(i, j)[1];
                int r = gt_color.at<cv::Vec3b>(i, j)[2];
                if (r > 0)
                {
                    if (b > 0) gt_gray.at<uchar>(i,j)=1;    // positive
                    else gt_gray.at<uchar>(i,j)=0;          // negative
                }
                else gt_gray.at<uchar>(i,j)=255;            // ignore index

            }
        }
        gt_gray.copyTo(m_image_gt);
    }
}

void Road::readLidarData()
{
    int i;
    float *p;
    char filename[128];
    pcl::PointXYZI point;
    sprintf(filename, "%s/velodyne/%s_%06d.bin", m_str_dir, m_str_type, m_frameID);
    memset(m_pLidar, 0, sizeof(float)*MAX_LIDAR_POINTS_NUM);
    FILE *fp = NULL;
    if ((fp = fopen(filename, "rb")) == NULL)
    {
        printf("can not open file %s !\n", filename);
        exit(-1);
    }
    m_nPoints = fread(m_pLidar, sizeof(float), MAX_LIDAR_POINTS_NUM, fp) / 4;
    /// read bin to pcl PointCloud
    m_pointCloud.clear();
    for (i = 0, p = m_pLidar; i < m_nPoints; i++, p += 4)
    {
        point.x=*p;
        point.y=*(p+1);
        point.z=*(p+2);
        point.intensity=*(p+3);
        m_pointCloud.push_back(point);
    }
    fclose(fp);
}


void Road::computeNormal(pcl::PointCloud<pcl::Normal> &normal, int k)
{
    normal.clear();
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(m_pointCloud.makeShared());
    ne.setInputCloud(m_pointCloud.makeShared());
    ne.setSearchMethod(tree);
    ne.setKSearch(k);
//    ne.setRadiusSearch(radius);
    ne.compute(normal);
}


void Road::projectLidar2Image()
{
    int idx, x, y, count=0;
    cv::Mat p_l,p_c,p_p;
//    cv::Mat height(m_height, m_width, CV_8U, cv::Scalar(0));
    cv::Mat im_x(m_height, m_width, CV_8U, cv::Scalar(0));
    cv::Mat im_y(m_height, m_width, CV_8U, cv::Scalar(0));
    cv::Mat im_z(m_height, m_width, CV_8U, cv::Scalar(0));

    cv::Mat indices(m_height, m_width,CV_32S, cv::Scalar(0));
    for(idx=0;idx<m_pointCloud.size();idx++)
    {
        pcl::PointXYZI point=m_pointCloud[idx];
        p_l = (cv::Mat_<float>(3, 1) << point.x,point.y,point.z);
        if (p_l.at<float>(0, 0) < 80 && p_l.at<float>(0, 0) > 5)      //select region in camera view
        {
            p_c = m_Tr_velo_to_cam.colRange(0, 3)*p_l + m_Tr_velo_to_cam.col(3);     //to camera
            p_p = m_P2.colRange(0,3)*m_R0_rect*p_c+m_P2.col(3);       //to pixel

            x = cvRound(p_p.at<float>(0, 0) / p_p.at<float>(2, 0));
            y = cvRound(p_p.at<float>(1, 0) / p_p.at<float>(2, 0));
            // if the point is in the image plane
            if (x >= 0 && x < m_width && y >= 0 && y < m_height)
            {
                indices.at<int>(y,x)=idx;
                im_x.at<uchar>(y, x) = uchar(cv::min(cv::max(0,int((point.x - 6.0) * 255 / 40.0)), 255));
                im_y.at<uchar>(y, x) = uchar(cv::min(cv::max(0,int((point.y + 10.0) * 255 / 20.0)), 255));
                im_z.at<uchar>(y, x) = uchar(cv::min(cv::max(0,int((point.z + 2.0) * 255 / 4.0)), 255));
//                point.x= point.x<6?6:(point.x>46?46:point.x);       // 6m~46m
//                point.y= point.y<-10?-10:(point.y>10?10:point.y);   // -10m~10m
//                point.z= point.z<-2?-2:(point.z>1?1:point.z);       // -2m~2m
//                point.x = (point.x-6)/40.;
//                point.y = (point.y+10)/20.;
//                point.z = (point.z+2)/3.;
                count++;
            }
        }
    }
    indices.copyTo(m_Indices);
//    cv::imshow("indices", indices);
//    cv::waitKey(0);

    im_x.copyTo(m_sparse_x);
//    cv::imshow("x", m_sparse_x);
//    cv::waitKey(0);
    im_y.copyTo(m_sparse_y);
//    cv::imshow("y", m_sparse_y);
//    cv::waitKey(0);
    im_z.copyTo(m_sparse_z);
//    cv::imshow("z", m_sparse_z);
//    cv::waitKey(0);
//    printf("valid point num is %d\n",count);
}

void Road::saveXYZINormalAsPCDFile()
{
    char filename[128];
    sprintf(filename, "%s/velodyne_pcd/%s_%06d.pcd", m_str_dir, m_str_type, m_frameID);
    pcl::PointCloud<pcl::PointXYZINormal> cloud_with_normals;
    pcl::concatenateFields(m_pointCloud, m_normal2, cloud_with_normals);
    pcl::io::savePCDFile("test.pcd",cloud_with_normals,true);
    printf("save %s succeed!\n",filename);
}

void Road::saveXYZRGBNormalAsPCDFile()
{
    int i,j;
    char filename[128];
    sprintf(filename, "%s/velodyne_image_pcd/%s_%06d.pcd", m_str_dir, m_str_type, m_frameID);
    pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
    for(i=0;i<m_height;i++)
    {
        for(j=0;j<m_width;j++)
        {
            int idx=m_Indices.at<int>(i,j);
            if(idx>0)
            {
                pcl::PointXYZRGBNormal point_rgb_normal;
                pcl::PointXYZI point=m_pointCloud[idx];
                pcl::Normal normal=m_normal2[idx];
                cv::Vec3b color=m_image_l.at<cv::Vec3b>(i,j);
                point_rgb_normal.x=point.x;
                point_rgb_normal.y=point.y;
                point_rgb_normal.z=point.z;
                point_rgb_normal.b=color(0);
                point_rgb_normal.g=color(1);
                point_rgb_normal.r=color(2);
                point_rgb_normal.normal_x=normal.normal_x;
                point_rgb_normal.normal_y=normal.normal_y;
                point_rgb_normal.normal_z=normal.normal_z;
                point_rgb_normal.curvature=normal.curvature;
                cloud.push_back(point_rgb_normal);
            }
        }
    }
    pcl::io::savePCDFile(filename,cloud,true);
    printf("save %s succeed!\n",filename);
}


void Road::saveUVXYZINormalRGBLabelAsTxtFile()
{
    int i,j;
    char filename[128];
    sprintf(filename, "%s/velodyne_image/%s_%06d.txt", m_str_dir, m_str_type, m_frameID);
    FILE *fp=fopen(filename,"wt");
    assert(fp);
    for(i=0;i<m_height;i++)
    {
        for(j=0;j<m_width;j++)
        {
            int idx=m_Indices.at<int>(i,j);
            if(idx>0)
            {
                pcl::PointXYZI point=m_pointCloud[idx];
                pcl::Normal normal=m_normal2[idx];
                cv::Vec3b color=m_image_l.at<cv::Vec3b>(i,j);
                if(m_isTrain)
                    fprintf(fp,"%d %d %f %f %f %f %f %f %f %f %f %f %f %d\n",
                            i,j,  // row,col
                            point.x,point.y,point.z,point.intensity,   // XYZI
                            normal.normal_x,normal.normal_y,normal.normal_z,normal.curvature,   // NC
                            color(2)/255., color(1)/255., color(0)/255.,  // BGR -> RGB
                            m_image_gt.at<uchar>(i,j));    // label
                else
                    fprintf(fp,"%d %d %f %f %f %f %f %f %f %f %f %f %f\n",
                            i,j,  // row,col
                            point.x,point.y,point.z,point.intensity,   // XYZI
                            normal.normal_x,normal.normal_y,normal.normal_z,normal.curvature,   // NC
                            color(2)/255.,color(1)/255.,color(0)/255.);  // BGR -> RGB
            }
        }
    }
    printf("save %s succeed!\n",filename);
}


void Road::saveUVXYZMultiNormalRGBLabelAsTxtFile()
{
    int i,j;
    char filename[128];
    sprintf(filename, "%s/velodyne_normal/%s_%06d.txt", m_str_dir, m_str_type, m_frameID);
    FILE *fp=fopen(filename,"wt");
    assert(fp);
    for(i=0;i<m_height;i++)
    {
        for(j=0;j<m_width;j++)
        {
            int idx=m_Indices.at<int>(i,j);
            if(idx>0)
            {
                pcl::PointXYZI point=m_pointCloud[idx];
                pcl::Normal normal1=m_normal1[idx];
                pcl::Normal normal2=m_normal2[idx];
                pcl::Normal normal3=m_normal3[idx];
                cv::Vec3b color=m_image_l.at<cv::Vec3b>(i,j);
                point.x= point.x<6?6:(point.x>46?46:point.x);       // 6m~46m
                point.y= point.y<-10?-10:(point.y>10?10:point.y);   // -10m~10m
                point.z= point.z<-2?-2:(point.z>1?1:point.z);       // -2m~2m
                point.x = (point.x-6)/40.;
                point.y = (point.y+10)/20.;
                point.z = (point.z+2)/3.;
                if(m_isTrain)
                    fprintf(fp,"%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n",
                            i,j,  // row,col
                            point.x,point.y,point.z,point.intensity,   // XYZI
                            color(2)/255.,color(1)/255.,color(0)/255.,  //RGB
                            normal1.normal_x,normal1.normal_y,normal1.normal_z,normal1.curvature,
                            normal2.normal_x,normal2.normal_y,normal2.normal_z,normal2.curvature,
                            normal3.normal_x,normal3.normal_y,normal3.normal_z,normal3.curvature,
                            m_image_gt.at<uchar>(i,j));    // label
                else
                    fprintf(fp,"%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
                            i,j,  // row,col
                            point.x,point.y,point.z, point.intensity,   // XYZ
                            color(2)/255.,color(1)/255.,color(0)/255.,  //RGB
                            normal1.normal_x,normal1.normal_y,normal1.normal_z,normal1.curvature,
                            normal2.normal_x,normal2.normal_y,normal2.normal_z,normal2.curvature,
                            normal3.normal_x,normal3.normal_y,normal3.normal_z,normal3.curvature);
            }
        }
    }
    printf("save %s succeed!\n",filename);
}


void Road::saveGTImage()
{
    char filename[128];
    sprintf(filename, "%s/gt_2/%s_%06d.png", m_str_dir, m_str_type, m_frameID);
    cv::imwrite(filename, m_image_gt);
    printf("save %s succeed!\n",filename);
}


void Road::saveSparseCloudImages()
{
    char filename[128];
    sprintf(filename, "%s/sparse/x/%s_%06d.png", m_str_dir, m_str_type, m_frameID);
    cv::imwrite(filename, m_sparse_x);
    sprintf(filename, "%s/sparse/y/%s_%06d.png", m_str_dir, m_str_type, m_frameID);
    cv::imwrite(filename, m_sparse_y);
    sprintf(filename, "%s/sparse/z/%s_%06d.png", m_str_dir, m_str_type, m_frameID);
    cv::imwrite(filename, m_sparse_z);
    printf("save %s succeed!\n",filename);
}

void Road::Initialize(const char *dir, const char *type, bool isTrain)
{
    m_isTrain = isTrain;
    if(m_isTrain)
        sprintf(m_str_dir, "%s/training", dir);
    else
        sprintf(m_str_dir, "%s/testing", dir);
    sprintf(m_str_type, "%s", type);
    m_pLidar = new float[MAX_LIDAR_POINTS_NUM * sizeof(float)];
    m_P0.create(3, 4, CV_32F);
    m_P1.create(3, 4, CV_32F);
    m_P2.create(3, 4, CV_32F);
    m_P3.create(3, 4, CV_32F);
    m_R0_rect.create(3, 3, CV_32F);
    m_Tr_velo_to_cam.create(3, 4, CV_32F);
    m_Tr_imu_to_velo.create(3, 4, CV_32F);
    m_Tr_cam_to_road.create(3, 4, CV_32F);
}


void Road::DoNext(int frameID)
{
    m_frameID = frameID;
    readCalibrationFile();
    readImages();
    readLidarData();
//    computeNormal(m_normal1, 5);
    computeNormal(m_normal2, 15);
//    computeNormal(m_normal3, 25);
    projectLidar2Image();
//    saveXYZRGBNormalAsPCDFile();
//    saveXYZINormalAsPCDFile();
    saveUVXYZINormalRGBLabelAsTxtFile();
//    saveUVXYZMultiNormalRGBLabelAsTxtFile();
//    saveGTImage();
    saveSparseCloudImages();
}

void Road::Debug()
{
    /// simple cloud viewer
//    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
//    viewer.showCloud(m_pointCloud.makeShared());
//    while(!viewer.wasStopped()){}
    /// pcl viewer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZI>(m_pointCloud.makeShared(),"sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    while(!viewer->wasStopped())
    {
//        viewer->spinOnce(100);
        viewer->spin();
    }

}