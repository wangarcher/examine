#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>
#include <list>
#include <ctime>
#include <stdlib.h>
#include <time.h>

#include <sys/time.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>

#include <pthread.h>
#include <signal.h>
#include <unistd.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>

#include "../include/Localizate.h"
#include "../include/ProcessCom.h"
#include "../include/Robot.h"
#include "../include/Gps.h"
#include "../include/Imu.h"
#include "../include/Odom.h"
#include "../include/Lmath.h"
#include "../include/Mtime.h"
#include "../include/Filter.h"



typedef struct _SendDataPack
{
     float x;
     float y;
     float z;
     float roll;
     float pitch;
     float yaw;
}SendDataPack;

typedef struct _RecvDataPack
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
}RecvDataPack;

//线程锁
pthread_mutex_t mutex;

//时间戳

//进程之间通信初始化
int InitProcessComunication()
{
    int processInitFlag;

    //processInitFlag = RegisterRobotPosInfo();//注册DBUS总线

    processInitFlag = InitSem(PosSemId, PosSemKey, 1, IPC_CREAT|0777); //初始化信号量

    //processInitFlag = MessageQueueInit(PosMsgId, PosMsgKey, IPC_CREAT|0777);  //初始化消息队列

    processInitFlag = InitShareMemory(PosShmId, PosShmKey,4096, IPC_CREAT|0777);//初始化共享内存

    return processInitFlag;
}

void InitLocalizate()
{
    int processComInitFlag = InitProcessComunication();//进程通信初始化 指定的大小必须是当前系统架构指定的大小必须是当前系统架构   

    if(processComInitFlag == -1)
    {
        std::cout << "进程通信初始化失败，请检查！" << std::endl;
    }

    //初始化卡尔曼滤波器
    
    int GpsRetVal    = 1; //对传感器进行初始化
    int ImuRetVal    = 1;
    int OdomRetVal   = 1;

    //  GpsRetVal = gps.InitGPS();
	// while( 4 != gps.getNowGpsQual())
	// {
	// 	usleep(1000 * 1000);
	// }
    ImuRetVal = imu.InitImu();
    OdomRetVal = odom.InitOdom();

    if (GpsRetVal == 0)
    {
        
    }	
}

void RTLocation(void* arg)
{
    V_sem(PosSemId, 0);

    int recvfd=open("../../../better_location", O_RDONLY); //TODO
    int sendfd=open("../../../former_location", O_WRONLY); //TODO

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
        float odom_v, odom_w;
        float robot_x, robot_y, robot_z, new_x, new_y;
        float robot_vx, robot_vy, robot_vz;
        float robot_roll, robot_pitch, robot_yaw, new_yaw;
        float imu_vx, imu_vy, imu_w;

    while(1)
    {

        kalmanFilter.setNowTimeStamp(getSysTime());

        if(!kalmanFilter.GetIsInitialized()) //判断滤波器是否初始化
        {
            kalmanFilter.Initialization();  
            continue;     
        }
        
        long int lastTimeStamp, nowTimeStamp;
        kalmanFilter.getLastTimeStamp(lastTimeStamp);
        kalmanFilter.getNowTimeStamp(nowTimeStamp);
        long int deltaTime = nowTimeStamp - lastTimeStamp;
        float d_t =  deltaTime * 0.0001;
        std::cout << d_t << std::endl;
        kalmanFilter.setLastTimeStamp(nowTimeStamp);



        odom.GetSpeed(odom_v, odom_w);
        imu.GetGravityXYZ(imu_vx, imu_vy, imu_w);

        std::cout << odom_v << ", " << odom_w  <<", " << imu_w << std::endl; 

        Eigen::VectorXf z_in(3,1);
        z_in << odom_v, odom_w, imu_w;

        kalmanFilter.Prediction(d_t);
        kalmanFilter.KFUpdate(z_in);

        Eigen::VectorXf x_out;
        x_out = kalmanFilter.GetX();

        SendDataPack senddatapack;
        memset(&senddatapack,0,sizeof(senddatapack));
        senddatapack.x = x_out[0];
        senddatapack.y = x_out[1];
        senddatapack.z = robot_z;
        senddatapack.roll = robot_roll;
        senddatapack.pitch = robot_pitch;
        senddatapack.yaw = LimitRadianRange(x_out[2]);
        write(sendfd,&senddatapack,sizeof(senddatapack));


        RecvDataPack recvdatapack;
        memset(&recvdatapack,0,sizeof(recvdatapack));
        read(recvfd,&recvdatapack,sizeof(recvdatapack));
        robot_x = recvdatapack.x;
        robot_y = recvdatapack.y;
        robot_z = recvdatapack.z;
        robot_roll = recvdatapack.roll;
        robot_pitch = recvdatapack.pitch;
        robot_yaw = LimitRadianRange(recvdatapack.yaw);

        Eigen::VectorXf x_refine(5,1);
        x_refine << robot_x, robot_y, robot_yaw, x_out[3], x_out[4];
        kalmanFilter.SetX(x_refine);
        

        //new_x = 76 - robot_y/0.25;
        //new_y = 72 - robot_x/0.25;
        //new_yaw = -robot_yaw;
        robot.SetRobotPosition(robot_x, robot_y, robot_z);
        robot.SetRobotRotation(robot_roll, robot_pitch, robot_yaw);
                                  
        void* posMemory = AttachShareMemory(PosShmId); //链接共享内存
        if(posMemory == (void*) -1)
        {
            fprintf(stderr, "shmget failed\n");
            std::cout << "Attach " << std::endl;
            exit(EXIT_FAILURE);            
        }

        robot.GetRobotPosition(robot_x, robot_y, robot_z);
        robot.GetRobotVelocity(robot_vx, robot_vy, robot_vz);
        robot.GetRobotRotation(robot_roll, robot_pitch, robot_yaw);
        
        RobotPosShareMemory robotPosShareMemory; //获取机器人位姿

        robotPosShareMemory.robotPosTimeStamp = getSysTime(); //赋值时间戳
        robotPosShareMemory.robotPosition.x = robot_x; //赋值位姿
        robotPosShareMemory.robotPosition.y = robot_y;
        robotPosShareMemory.robotPosition.z = 0;
        robotPosShareMemory.robotPosition.roll = robot_roll;
        robotPosShareMemory.robotPosition.pitch = robot_pitch;
        robotPosShareMemory.robotPosition.yaw = robot_yaw;

        std::cout<<"robotPosShareMemory.robotPosTimeStamp="<<robotPosShareMemory.robotPosTimeStamp<<std::endl;
        std::cout<<"robotPosShareMemory.robotPosition.x="<<robotPosShareMemory.robotPosition.x<<std::endl;
        std::cout<<"robotPosShareMemory.robotPosition.y="<<robotPosShareMemory.robotPosition.y<<std::endl;
        std::cout<<"robotPosShareMemory.robotPosition.yaw="<<robotPosShareMemory.robotPosition.yaw<<std::endl;
		
        P_sem(PosSemId, 0);
        WriteShareMemory(posMemory, &robotPosShareMemory, sizeof(RobotPosShareMemory));
        V_sem(PosSemId, 0);

        int ret = DisattachShareMemory(posMemory);//共享内存去链接
        if(ret != 0) 
        {
            std::cout << "Failed detached memory" << std::endl;
        }
        usleep(5000);
    }
}

void DestoryLocalizate()
{
    DestorySem(PosSemId);
    DestoryMessageQueue(PosMsgId);
    DestoryShareMemory(PosShmId);
}

int main()
{
    InitLocalizate();

    RTLocation(NULL);

    DestoryLocalizate();

    return 0;
}
