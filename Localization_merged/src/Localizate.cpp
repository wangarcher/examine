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

float odom_v, odom_w;
float robot_x, robot_y, robot_z;
float robot_vx, robot_vy, robot_vz;
float robot_roll, robot_pitch, robot_yaw, new_yaw;
float imu_vx, imu_vy, imu_w;
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

    //GpsRetVal = gps.InitGPS();
	//while( 4 != gps.getNowGpsQual())
	//{
	//    usleep(1000 * 1000);
	//}
    ImuRetVal = imu.InitImu();
    OdomRetVal = odom.InitOdom();

    if (GpsRetVal == 0)
    {
        
    }	
}

void PriorLocation(float& prior_x, float& prior_y, float& prior_z,
                   float& prior_roll, float& prior_pitch, float& prior_yaw,
                   float& prior_v, float& prior_w)
{
    kalmanFilter.setNowTimeStamp(getSysTime());
    while(!kalmanFilter.GetIsInitialized()) //判断滤波器是否初始化
    {
        kalmanFilter.Initialization();   
    }
        
    long int lastTimeStamp, nowTimeStamp;
    kalmanFilter.getLastTimeStamp(lastTimeStamp);
    kalmanFilter.getNowTimeStamp(nowTimeStamp);
    long int deltaTime = nowTimeStamp - lastTimeStamp;
    float d_t =  deltaTime * 0.001;
    std::cout << d_t << std::endl;
    kalmanFilter.setLastTimeStamp(nowTimeStamp);

    odom.GetSpeed(odom_v, odom_w);
    imu.GetGravityXYZ(imu_vx, imu_vy, imu_w);
    std::cout << "[odom_v odom_w imu_w]: " << odom_v << ", " << odom_w  <<", " << imu_w << std::endl; 

    Eigen::VectorXf z_in(3,1);
    z_in << odom_v, odom_w, imu_w;

    kalmanFilter.Prediction(d_t);
    kalmanFilter.KFUpdate(z_in);

    Eigen::VectorXf x_out;
    x_out = kalmanFilter.GetX();
    prior_x = x_out[0];
    prior_y = x_out[1];
    prior_z = robot_z;
    prior_roll = robot_roll;
    prior_pitch = robot_pitch;
    prior_yaw = LimitRadianRange(x_out[2]);
    prior_v = x_out[3];
    prior_w = x_out[4];
}

void PosteriorLocation(float& post_x, float& post_y, float& post_z,
                       float& post_roll, float& post_pitch, float& post_yaw,
                       float& post_v, float& post_w)
{
    robot_x = post_x;
    robot_y = post_y;
    robot_z = post_z;
    robot_roll = post_roll;
    robot_pitch = post_pitch;
    robot_yaw = LimitRadianRange(post_yaw);

    Eigen::VectorXf x_refine(5,1);
    x_refine << robot_x, robot_y, robot_yaw, post_v, post_w;
    kalmanFilter.SetX(x_refine);

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
    robotPosShareMemory.robotPosition.v = post_v;
    robotPosShareMemory.robotPosition.w = post_w;
    std::cout << "YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY" << std::endl;
    std::cout<<"robotPosShareMemory.robotPosTimeStamp=" <<robotPosShareMemory.robotPosTimeStamp <<std::endl;
    std::cout<<"robotPosShareMemory.robotPosition.x="   <<robotPosShareMemory.robotPosition.x   <<std::endl;
    std::cout<<"robotPosShareMemory.robotPosition.y="   <<robotPosShareMemory.robotPosition.y   <<std::endl;
    std::cout<<"robotPosShareMemory.robotPosition.yaw=" <<robotPosShareMemory.robotPosition.yaw <<std::endl;
    std::cout<<"robotPosShareMemory.robotPosition.v="   <<robotPosShareMemory.robotPosition.v   <<std::endl;
    std::cout<<"robotPosShareMemory.robotPosition.w="   <<robotPosShareMemory.robotPosition.w   <<std::endl;

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


void DestoryLocalizate()
{
    DestorySem(PosSemId);
    DestoryMessageQueue(PosMsgId);
    DestoryShareMemory(PosShmId);
}


