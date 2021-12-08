#include "road.h"

using namespace std;

#if 0
KITTI::Odometry odometry;
int main()
{
    odometry.Initialize("/media/yangfei/Repository/KITTI/odometry","00");

    for (int frameID = 0; frameID < 4541 ; frameID++)
    {
        cout<<"frame: "<<frameID<<endl;
        odometry.DoNext(frameID);
    }
//    odometry.Evaluate("/home/yangfei/Workspace/ORB_SLAM2/Examples/RGB-D/Trajectory.txt","/media/yangfei/Repository/KITTI/odometry/dataset/poses/00.txt");
    std::cout << "process over !" << std::endl;
    return 0;
}
#endif

Road road;
int main()
{
    bool isTrain= false;
    const char *dir="/media/yangfei/Repository/KITTI/data_road";
    const char *type="uu";
    road.Initialize(dir,type,isTrain);
    for (int frameID = 0; frameID < 100; frameID++)
    {
        cout<<"frame: "<<frameID<<endl;
        road.DoNext(frameID);
    }
    std::cout << "process over !" << std::endl;
    return 0;
}