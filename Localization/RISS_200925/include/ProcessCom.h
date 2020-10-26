#ifndef _PROCESSCOM_H
#define _PROCESSCOM_H

#include <sys/msg.h>
#include <sys/types.h>
#include <sys/ipc.h> 

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../include/Odom.h"
#include "../include/Imu.h"
#include "../include/Gps.h"
#include "../include/Robot.h"

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

void six2Trans(float yaw, float roll, float pitch, float x, float y, float z, Eigen::Matrix4f& transform);


bool RegisterRobotPosInfo();
bool SendRobotPosInfo();

/*************** 信号量 ***********************/
extern int PosSemKey;
extern int PosSemId; //信号量
/*************** 信号量 ***********************/

/*************** 消息队列通信 ***********************/
extern int PosMsgId;         //定义位置消息Id, 进程间消息队列共享
extern int PosMsgKey;  //消息键
/*************** 消息队列通信 ***********************/

/*************** 共享内存通信 ***********************/
extern int PosShmKey; 
extern int PosShmId;
/*************** 共享内存通信 ***********************/


#endif 
