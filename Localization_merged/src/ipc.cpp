#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <dbus/dbus.h>
#include <stdbool.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sys/shm.h>
#include <sys/types.h> 
#include <sys/msg.h>
#include <sys/sem.h>
#include <sys/ipc.h>

#include <errno.h>

#include "../include/ipc.h"

/***********************************  组合导航定位模块进程间通信 ****************************/
/*************** 定位模块的IPC信号量 ***********************/
int INPosSemKey = 2463;
int INPosSemId; 
/*************** 定位模块的IPC信号量 ***********************/

/*************** 定位模块的消息队列通信 ***********************/
int INPosMsgKey = 2464;  //消息键
int INPosMsgId;       //定义位置消息Id, 进程间消息队列共享
/*************** 定位模块的消息队列通信 ***********************/

/*************** 定位模块的共享内存通信 ***********************/
int INPosShmKey = 2465;
int INPosShmId; //获取共享内存(Odom Imu Gps + Pos)句柄  
/*************** 定位模块的共享内存通信 ***********************/
/***********************************  组合导航定位模块进程间通信 ******************************/


/***********************************  SLAM定位模块进程间通信 ****************************/
/*************** 定位模块的IPC信号量 ***********************/
int LocalizationPosSemKey = 2466;
int LocalizationPosSemId; 
/*************** 定位模块的IPC信号量 ***********************/

/*************** 定位模块的消息队列通信 ***********************/
int LocalizationPosMsgId;         //定义位置消息Id, 进程间消息队列共享
int LocalizationPosMsgKey = 2467;  //消息键
/*************** 定位模块的消息队列通信 ***********************/

/*************** 定位模块的共享内存通信 ***********************/
int LocalizationPosShmKey = 2468;
int LocalizationPosShmId; 
/*************** 定位模块的共享内存通信 ***********************/
/***********************************  SLAM定位模块进程间通信 ******************************/


/***********************************  视觉模块进程间通信 ******************************/
/*************** 视觉模块的IPC信号量 ***********************/
int VisualOccPosSemKey = 2469;
int VisualOccPosSemId; 
/*************** 视觉模块的IPC信号量 ***********************/

/*************** 视觉模块的消息队列通信 ***********************/
int VisualOccPosMsgKey = 2470;  //消息键
int VisualOccPosMsgId;         //定义位置消息Id, 进程间消息队列共享
/*************** 视觉模块的消息队列通信 ***********************/

/*************** 视觉模块的共享内存通信 ***********************/
int VisualOccPosShmKey = 2471;
int VisualOccPosShmId; 
/*************** 视觉模块的共享内存通信 ***********************/
/***********************************  视觉模块进程间通信 *****************************/

/***********************************  雷达模块进程间通信 ******************************/
/*************** 雷达模块的IPC信号量 ***********************/
int LidarOccPosSemKey = 2472;
int LidarOccPosSemId; 
/*************** 雷达模块的IPC信号量 ***********************/

/*************** 雷达模块的消息队列通信 ***********************/
int LidarOccPosMsgKey = 2473;  //消息键
int LidarOccPosMsgId;         //定义位置消息Id, 进程间消息队列共享
/*************** 雷达模块的消息队列通信 ***********************/

/*************** 雷达模块的共享内存通信 ***********************/
int LidarOccPosShmKey = 2474;
int LidarOccPosShmId; 
/*************** 雷达模块的共享内存通信 ***********************/
/***********************************  雷达模块进程间通信 *****************************/


