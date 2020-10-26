/*

Project: icp alignment for localization
Date: 20/08/20
@Author: ALL
Detail: align two clouds in the localization, 
        so as to get the global pose of the current frame
Scenario: localization

*/

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/transformation_estimation_lm.h>

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <vector>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>

#include <string.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <iostream>

#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "../include/Lidar.h"
#include "../include/ipc.h"


using namespace std;
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormal;


struct information_of_barrier
{
    float centerx;
    float centery;
};

int num_of_barrier;


struct Posemat
{
    Eigen::Matrix4f global_transform;
};


typedef struct _RecvDataPack
{
    int index;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
}RecvDataPack;

typedef struct _SendDataPack
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
}SendDataPack;
//uniform sampling func
//data acquired: Cloud's ptr for processing.
void uniform_sampling (PointCloud::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::UniformSampling<pcl::PointXYZ> US;
    US.setInputCloud(cloud);
    US.setRadiusSearch(0.2f);
    US.filter(*cloud_filtered);
    *cloud = *cloud_filtered;

}

//remove singel func
//data acquired: Cloud's ptr for processing.
void remove_single (PointCloud::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> RS;
    RS.setInputCloud(cloud);
    RS.setRadiusSearch(0.5);
    RS.setMinNeighborsInRadius(4);
    RS.filter(*cloud_filtered);
    *cloud = *cloud_filtered;
}

//another new filter to remove the point faraway from the origin
//added on 7/30 by Wang
//data acquired: pcl::PointCloud<PointNormal> PointCloudN
void remove_distant (PointCloud::Ptr cloud)
{
    PointCloud::Ptr cloud_filtered(new PointCloud);
    pcl::StatisticalOutlierRemoval<PointT> RD;
    RD.setInputCloud(cloud);    
    RD.setMeanK(50);
    RD.setStddevMulThresh(1.0);
    RD.filter(*cloud_filtered);
    *cloud = *cloud_filtered;

}


//icp point align algorithm
//data acquired: 1 & 2 is the Cloud's ptr for aligning, 3 is the Cloud's ptr for output, 4 is transformation matrix, 5 is unclear.
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f&final_transform)
{
    int iterations = 100;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_tgt);
    icp.setTransformationEpsilon(1e-15);
    icp.setMaxCorrespondenceDistance(2);
    icp.setEuclideanFitnessEpsilon(0.001);
    icp.setMaximumIterations(iterations);
    icp.align(*output);
    final_transform = icp.getFinalTransformation();
}

// func from raw pose values to the eigen transformation mat
void six2Trans(float roll, float pitch, float yaw, float x, float y, float z, Eigen::Matrix4f& transform)
{
    Eigen::Vector3f eulerAngle(yaw, pitch, roll);
    Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(eulerAngle(2),Eigen::Vector3f::UnitX()));
    Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(eulerAngle(1),Eigen::Vector3f::UnitY()));
    Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(eulerAngle(0),Eigen::Vector3f::UnitZ()));

    Eigen::Matrix3f rotation;
    rotation = yawAngle * pitchAngle * rollAngle;

    Eigen::Translation3f translation(x, y, z);
    transform = (translation * rotation).matrix();
}


void quaterniond2eulerangle(Eigen::Quaternionf& q, float& roll, float& pitch, float& yaw)
{
    double sinr_cosp = +2.0*(q.w()*q.x() + q.y()*q.z());
    double cosr_cosp = +1.0 - 2.0*(q.x()*q.x() + q.y()*q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = +2.0*(q.w()*q.y() - q.z()*q.x());
    if(fabs(sinp) >= 1) pitch = copysign(M_PI/2, sinp);
    else pitch = asin(sinp);

    double siny_cosp = +2.0*(q.w()*q.z() + q.x()*q.y());
    double cosy_cosp = +1.0 - 2.0*(q.y()*q.y() + q.z()*q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}



vector<information_of_barrier> remove_duplicates( vector<vector<float>> &data)
{
    for(int i = 0; i < data.size(); i++)
    {
        data[i][0] = (float)round(data[i][0]*4);
        data[i][1] = (float)round(data[i][1]*4);
    }
    sort(data.begin(),data.end());
    data.erase(unique(data.begin(), data.end()),data.end());
    std::vector<information_of_barrier> Information_of_barrier;
    for(int i = 0; i < data.size(); i++)
    {
        if(data[i][0]<=12&&data[i][0]>=0&&data[i][1]<=8&&data[i][1]>=-8){
        information_of_barrier barrier;
        barrier.centerx = data[i][0]/4;
        barrier.centery = data[i][1]/4;
        Information_of_barrier.push_back(barrier);
        }
    
    }
    return Information_of_barrier;
}

int main(int argc, char** argv)
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;

    //processInitFlag = InitSem(LidarInSemId, LidarInSemKey, 1, IPC_CREAT|0777); //初始化信号量
    //processInitFlag = InitShareMemory(LidarInShmId, LidarInShmKey,4096, IPC_CREAT|0777);//初始化共享内存
    std::vector<std::vector<float>> data3d;
    std::vector<std::vector<float>> data_after;
    std::vector<float> data2d;

    int recvfd=open("../../../index", O_RDWR); //TODO
    int sendfd=open("../../../better_location", O_RDWR); //TODO


    
    if(recvfd<0)
    {
        std::cout<<"recv default"<<std::endl;
        exit(1);
    }
    if(sendfd<0)
    {
        std::cout<<"send default"<<std::endl;
        exit(1);
    }

    std::vector<Posemat> Pose;
     
    ifstream fin("../../../pose.txt"); // TODO
    int line = 0;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();

    while(!fin.eof())
    {
        fin >> line >> x >> y >> z >> yaw >> pitch >> roll;
        cout << " Receving No. " << line << " Pose." << endl;
        six2Trans(roll, pitch, yaw, x, y, z, GlobalTransform);
        Posemat a;
        a.global_transform = GlobalTransform;
        Pose.push_back(a);
        if (!fin.good()) break;
    }

    pthread_mutex_init(&LidarMutex,NULL);
    lidar.InitLidar();//start get lidar data
    sleep(3);

    InitSem(LidarOccPosSemId, LidarOccPosSemKey, 1, IPC_CREAT|0777); 
    MessageQueueInit(LidarOccPosMsgId, LidarOccPosMsgKey, IPC_CREAT|0777);
    InitShareMemory(LidarOccPosShmId, LidarOccPosShmKey,4096, IPC_CREAT|0777);

    V_sem(LidarOccPosSemId, 0);


    while(1)
    {
    PointCloud::Ptr source(new PointCloud);
    PointCloud::Ptr target(new PointCloud);
    PointCloud::Ptr temp(new PointCloud);
    PointCloud::Ptr Lidarcloud(new PointCloud);
        //updata lidar data get current cloud now
        //memcpy((void *)&target,&LidarSendCloudData.cloud, sizeof(target));
        
    pthread_mutex_lock(&LidarMutex);
    pcl::copyPointCloud(*safecloud, *Lidarcloud);
        //LidarSendCloudData.CloudReadSucceesFlag=0;
        //end updata lidar data

        for(int i = 0; i<Lidarcloud->size(); i++)
        {
            if(Lidarcloud->points[i].x >= 0.0 && Lidarcloud->points[i].x <= 3.0 && Lidarcloud->points[i].y >= -2 && Lidarcloud->points[i].y <= 2)
            {
                data2d.push_back(Lidarcloud->points[i].x);
                data2d.push_back(Lidarcloud->points[i].y);
                data3d.push_back(data2d);
                data2d.clear();
            }
        }
        std::vector<information_of_barrier> Information_of_barrier = remove_duplicates(data3d);
        //std::cout<<"*********"<<Information_of_barrier.size()<<std::endl;
        
        //链接内存
        void* lidarOccPosMemory;
        lidarOccPosMemory = AttachShareMemory(LidarOccPosShmId);

        if(lidarOccPosMemory == (void*)(-1))
        {
            std::cout << "Get occPosMemoy failed" << std::endl;
            return -1;      
        }

        P_sem(LidarOccPosSemId, 0);//使用互斥量
        int occNum = Information_of_barrier.size();
        //std::cout << "occNum" << occNum << std::endl;
        WriteShareMemory(lidarOccPosMemory, &occNum, sizeof(int));
        for(int loop = 0; loop < occNum; loop++)
        {
            WriteShareMemory((void*)(lidarOccPosMemory + sizeof(int) + loop * sizeof(information_of_barrier)), &Information_of_barrier[loop], sizeof(information_of_barrier));
        }
        V_sem(LidarOccPosSemId, 0);

        int ret = DisattachShareMemory(lidarOccPosMemory);
        if(ret != 0)
        {
            std::cout << "Failed detached memory" << std::endl;
        }



        //get the index from quad_tree
        RecvDataPack recvdatapack;
        memset(&recvdatapack,0,sizeof(recvdatapack));
        read(recvfd,&recvdatapack,sizeof(recvdatapack));



    //pseudo mat    
    Eigen::Matrix4f globalTransform_curr = Eigen::Matrix4f::Identity(); // for the result store
    Eigen::Matrix4f pairTransform        = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f priorTransform       = Eigen::Matrix4f::Identity(); 
    Eigen::Matrix4f relateTransform      = Eigen::Matrix4f::Identity();
    float min = 0;

    cout << "x: " <<recvdatapack.x << ", y: " << recvdatapack.y  << ", yaw: "<< recvdatapack.yaw << endl;
    six2Trans(recvdatapack.roll, recvdatapack.pitch, recvdatapack.yaw, 
              recvdatapack.x, recvdatapack.y, recvdatapack.z, priorTransform); 
  
    cout << "prior: \n" << priorTransform << endl;
    cout << "keyframe: \n" << Pose[recvdatapack.index].global_transform << endl;
    relateTransform = Pose[recvdatapack.index].global_transform.inverse() * priorTransform;
    cout << "relate: \n" << relateTransform << endl; 
    //x_diff = abs(relateTransform(0,3));
    //y_diff = abs(relateTransform(1,3));


    min = hypot(relateTransform(0,3), relateTransform(1,3));
    SendDataPack senddatapack;
    memset(&senddatapack,0,sizeof(senddatapack));
    if(min < 0.4)
    {
        //get the two clouds
        char namebuf[30];
        sprintf(namebuf, "../../../data/%d.pcd", recvdatapack.index+1); //
        pcl::io::loadPCDFile (namebuf, *source);        //get source cloud from keyframe inventory

        uniform_sampling(Lidarcloud);
        uniform_sampling(source);
        cout << "size after filter:" << Lidarcloud->size() << endl;

        pcl::transformPointCloud(*Lidarcloud, *target, relateTransform);
        //refine needed!!!
        pairAlign(target, source, temp, pairTransform);
        cout << "pair: \n" << pairTransform << endl;
        cout << "pair inverse: \n" <<  pairTransform.inverse() << endl;
        globalTransform_curr = Pose[recvdatapack.index].global_transform * pairTransform * relateTransform;
        cout << "ture: \n" << globalTransform_curr<< endl;

        //sent to local
        Eigen::Matrix3f rotation_matrix = globalTransform_curr.block<3,3>(0,0);
        Eigen::Quaternionf quaternion(rotation_matrix);
        float newr, newp, newy;
        quaterniond2eulerangle(quaternion, newr, newp, newy);

        senddatapack.x = globalTransform_curr(0,3);
        senddatapack.y = globalTransform_curr(1,3);
        senddatapack.z = globalTransform_curr(2,3);
        senddatapack.roll = newr;
        senddatapack.pitch = newp;
        senddatapack.yaw = newy;
        write(sendfd,&senddatapack,sizeof(senddatapack));
    }
    else
    {
        senddatapack.x = recvdatapack.x;
        senddatapack.y = recvdatapack.y;
        senddatapack,z = recvdatapack.z;
        senddatapack.roll = recvdatapack.roll;
        senddatapack.pitch = recvdatapack.pitch;
        senddatapack.yaw = recvdatapack.yaw;
        write(sendfd,&senddatapack,sizeof(senddatapack));
    }
    pthread_mutex_unlock(&LidarMutex);
        //close();
        //showPCD(result);
        
    }

}



