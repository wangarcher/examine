#ifndef _IPC_H
#define _IPC_H

#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sys/msg.h>
#include <sys/types.h>
#include <sys/ipc.h> 

typedef struct 
{
    float x;
	float y;
	float z;
    float roll;  //翻滚角
    float pitch; //俯仰角
    float yaw;   //偏航角
}RobotPos;

typedef struct 
{
    long int  robotPosTimeStamp;
    RobotPos  robotPosition; 
}RobotPosShareMemory;

union semun{ 
    int val; 
    struct semid_ds *buf; 
    unsigned short *arry; 
};

void InitIPC();

int InitSem(int &semId, int key, int num, int cmd);
int CreatSem(int &semId, int key, int num, int cmd);
int SetSem(int semId,int val);
int GetSem(int semId, int& val);
int P_sem(int semId, int semIndex);
int V_sem(int semId, int semIndex);
int DestorySem(int semId);

int MessageQueueInit(int &msgId, int msgKey, int cmd);
int CreatMessageQueue(int &msgId, int msgKey, int cmd);
int SendMessageQueue(int msgId, void* dataAdress, int msgSize,int cmd);
int RcvMessageQueue(int msgId, void* dataAdress, int msgSize, int cmd);
int DestoryMessageQueue(int msgId);

int InitShareMemory(int &shmid, int key, int size, int cmd);
int CreatShareMemory(int &shmid, key_t key, int size, int cmd);
void* AttachShareMemory(int shmid);
void* WriteShareMemory(void* shmaddr, void* source, int size);
void* ReadShareMemory(void* destion, void* shmaddr, int size);
int DisattachShareMemory(void* shmaddr);
int DestoryShareMemory(int shmid);

/***********************************  组合导航定位模块进程间通信 ****************************/
/*************** 定位模块的IPC信号量 ***********************/
extern int INPosSemKey;
extern int INPosSemId; 
/*************** 定位模块的IPC信号量 ***********************/

/*************** 定位模块的消息队列通信 ***********************/
extern int INPosMsgKey;  //消息键
extern int  INPosMsgId;       //定义位置消息Id, 进程间消息队列共享
/*************** 定位模块的消息队列通信 ***********************/

/*************** 定位模块的共享内存通信 ***********************/
extern int INPosShmKey;
extern int INPosShmId; //获取共享内存(Odom Imu Gps + Pos)句柄  
/*************** 定位模块的共享内存通信 ***********************/
/***********************************  组合导航定位模块进程间通信 ******************************/

/***********************************  SLAM定位模块进程间通信 ****************************/
/*************** 定位模块的IPC信号量 ***********************/
extern int LocalizationPosSemKey;
extern int LocalizationPosSemId; 
/*************** 定位模块的IPC信号量 ***********************/

/*************** 定位模块的消息队列通信 ***********************/
extern int LocalizationPosMsgKey;  //消息键
extern int LocalizationPosMsgId;   //定义位置消息Id, 进程间消息队列共享

/*************** 定位模块的消息队列通信 ***********************/

/*************** 定位模块的共享内存通信 ***********************/
extern int LocalizationPosShmKey;
extern int LocalizationPosShmId; //获取共享内存(Odom Imu Gps + Pos)句柄  
/********************************** 定位模块的共享内存通信 ***********************/

/***********************************  SLAM定位模块进程间通信 ******************************/

/***********************************  视觉进程间通信 ********************************/
/*************** 视觉模块的IPC信号量 ***********************/
extern int VisualOccPosSemKey;
extern int VisualOccPosSemId; 
/*************** 视觉模块的IPC信号量 ***********************/

/*************** 视觉模块的消息队列通信 ***********************/
extern int VisualOccPosMsgKey;  //消息键
extern int VisualOccPosMsgId;         //定义位置消息Id, 进程间消息队列共享
/*************** 视觉模块的消息队列通信 ***********************/

/*************** 视觉模块的共享内存通信 ***********************/
extern int VisualOccPosShmKey;
extern int VisualOccPosShmId; //获取共享内存(Odom Imu Gps + Pos)句柄  
/*************** 视觉模块的共享内存通信 ***********************/
/***********************************  视觉进程间通信 *****************************/

/***********************************  雷达模块进程间通信 ******************************/
/*************** 雷达模块的IPC信号量 ***********************/
extern int LidarOccPosSemKey;
extern int LidarOccPosSemId; 
/*************** 雷达模块的IPC信号量 ***********************/

/*************** 雷达模块的消息队列通信 ***********************/
extern int LidarOccPosMsgKey;  //消息键
extern int LidarOccPosMsgId;         //定义位置消息Id, 进程间消息队列共享
/*************** 雷达模块的消息队列通信 ***********************/

/*************** 雷达模块的共享内存通信 ***********************/
extern int LidarOccPosShmKey;
extern int LidarOccPosShmId; 
/*************** 雷达模块的共享内存通信 ***********************/
/***********************************  雷达模块进程间通信 *****************************/

#endif
