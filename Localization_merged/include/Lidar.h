#pragma once

#ifndef _LIDAR_H
#define _LIDAR_H

#include <boost/thread/thread.hpp>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <string.h>

typedef struct _LIDARSENDCLOUDSTRUCT
{
    pcl::PointCloud<pcl::PointXYZ>    cloud;
    float x;
    float y;
    float theta;
    unsigned char                     CloudReadSucceesFlag;
}LidarSendCloudStruct;


class LidarData
{
private:

public:
    LidarData();
    ~LidarData();

    int InitLidar();
    int InitSocket();

    void LidarParsing();

    static void* ReceiveLidarData(void *arg); //接收雷达数据
    static void* ParsingLidarData(void *arg); //处理雷达数据
};

extern std::vector<std::vector<float> > lidarAngle;	
extern std::vector<std::vector<int> > lidarDist;	
extern std::vector<std::vector<int> > lidarInstensity;	


extern LidarData lidar;
extern LidarData* plidar;
extern LidarSendCloudStruct LidarSendCloudData;
extern pthread_mutex_t LidarMutex;


extern pcl::PointCloud<pcl::PointXYZ>::Ptr safecloud;


#endif


