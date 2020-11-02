#ifndef _IPC_H
#define _IPC_H

#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sys/msg.h>
#include <sys/types.h>
#include <sys/ipc.h> 
#include "../include/ipc.h"
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
