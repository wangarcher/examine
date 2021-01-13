// Created by finley on 2020/12/8.
// Copyright (c) 2020  All rights reserved.

//# -*- coding: utf-8 -*-
// @FileName: Information_share_out.cpp
// @Email: Guang.yang@i-usv.com
// @Software: CLion
// @Project: Localization

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
#include <stdbool.h>

#include <sys/shm.h>
#include <sys/types.h>
#include <sys/msg.h>
#include <sys/sem.h>
#include <sys/ipc.h>

#include <errno.h>

#include "../include/Information_share_out.h"
#include "../include/Para.h"

//*************** 二维码定位模块的信号量 ***********************/
int QRcodePosSemKey;
int QRcodePosSemId;
/*************** 二维码定位模块的信号量 ***********************/

/*************** 二维码定位模块的共享内存通信 ***********************/
int QRcodePosShmKey;
int QRcodePosShmId;
/*************** 二维码定位模块的共享内存通信 ***********************/

/*************** 雷达障碍物的信号量 ***********************/
int LidarObstacleSemKey;
int LidarObstacleSemId;
/*************** 雷达障碍物的信号量 ***********************/
/*************** 雷达障碍物的共享内存通信 ***********************/
int LidarObstacleShmKey;
int LidarObstacleShmId;
/*************** 雷达障碍物的共享内存通信 ***********************/

/*************** 视觉障碍物的信号量 ***********************/
int VisualObstacleSemKey;
int VisualObstacleSemId;
/*************** 视觉障碍物的信号量 ***********************/
/*************** 视觉障碍物的共享内存通信 ***********************/
int VisualObstacleShmKey;
int VisualObstacleShmId;
/*************** 视觉障碍物的共享内存通信 ***********************/

void InitIPC()
{
    const char* PwdQRcodeSemFile = PwdQRcodeSemPath.c_str();
    QRcodePosSemKey = ftok(PwdQRcodeSemFile, 0);
    InitSem(QRcodePosSemId, QRcodePosSemKey, 1, IPC_CREAT|0777);

    const char* PwdQRcodeShmFile = PwdQRcodeMemPath.c_str();
    QRcodePosShmKey = ftok(PwdQRcodeShmFile, 1);
    InitShareMemory(QRcodePosShmId, QRcodePosShmKey,4096, IPC_CREAT|0777);

    const char* PwdLidarObstacleSemFile = PwdLidarObstacleSemPath.c_str();
    LidarObstacleSemKey = ftok(PwdLidarObstacleSemFile, 2);
    InitSem(LidarObstacleSemId, LidarObstacleSemKey, 1, IPC_CREAT|0777);

    const char* PwdLidarObstacleShmFile = PwdLidarObstacleMemPath.c_str();
    LidarObstacleShmKey = ftok(PwdLidarObstacleShmFile, 3);
    InitShareMemory(LidarObstacleShmId, LidarObstacleShmKey, 4096, IPC_CREAT|0777);

    const char* PwdVisualObstacleSemFile = PwdVisualObstacleSemPath.c_str();
    VisualObstacleSemKey = ftok(PwdVisualObstacleSemFile,4);
    InitSem(VisualObstacleSemId, VisualObstacleSemKey, 1, IPC_CREAT|0777);

    const char* PwdVisualObstcaleShmFile = PwdVisualObstacleMemPath.c_str();
    VisualObstacleShmKey = ftok(PwdVisualObstcaleShmFile, 5);
    InitShareMemory(VisualObstacleShmId, VisualObstacleShmKey, 4096 * 4, IPC_CREAT|0777);
}
/***************************************************************  进程间的信号量通信 ***************************************************/
//(1)第一个参数key是长整型（唯一非零），系统建立IPC通讯 （ 消息队列、 信号量和 共享内存） 时必须指定一个ID值。
//通常情况下，该id值通过ftok函数得到，由内核变成标识符，要想让两个进程看到同一个信号集，只需设置key值不变就可以。
//(2)第二个参数nsem指定信号量集中需要的信号量数目
//(3)第三个参数flag是一组标志，当想要当信号量不存在时创建一个新的信号量，可以将flag设置为IPC_CREAT与文件权限做按位或操作。
//设置了IPC_CREAT标志后，即使给出的key是一个已有信号量的key，也不会产生错误。而IPC_CREAT | IPC_EXCL则可以创建一个新的，唯一的信号量，如果信号量已存在，返回一个错误。一般我们会还或上一个文件权限

int InitSem(int &semId, int key, int num, int cmd)
{
    semun sem_union;
    sem_union.val = 0;

    semId = semget(key_t(key), num, cmd);//创建并打开 或者创建 打开拆开 用时打开

    if(semId < 0)
    {
        perror("InitSem failed");
        return -1;

    }
        //信号量存在则获取
    else
    {
        std::cout << "InitSem Success" << std::endl;
        return 0;
    }

    if (semctl(semId, 0, SETVAL, sem_union) == -1) //设置信号量的计数值为0
    {
        printf("error to open semaphore!\n");
        return -1;
    }

    //设置信号量集合中的信号量（元素）的计数值
    SetSem(semId,1);

    return 0;
}

int CreatSem(int &semId, int key, int num, int cmd)
{
    semId = semget(key_t(key), num, cmd);
    if(semId < 0)
    {
        std::cout << "Creat Sem failed" << std::endl;
        return -1;
    }
    else
    {
        std::cout << "Creat Sem failed" << std::endl;
        return 0;
    }

}


//设置信号量集合里面的信号量的计数值val
int SetSem(int semId,int val)
{
    int ret = 0;

    if(semId < 0)
    {
        return -1;
    }
    semun su;
    su.val = val;
    ret = semctl(semId,0,SETVAL,su);
    return ret;
}

//获取信号量集合里面的信号量的计数值
int GetSem(int semId, int& val)
{
    int ret = 0;
    semun sem_union;
    sem_union.val = 0;

    if(semId < 0)
    {
        return -1;
    }

    ret = semctl(semId,0,GETVAL,sem_union);
    val = sem_union.val;
    return ret;
}


//struct sembuf{
//    short sem_num; // 除非使用一组信号量，否则它为0
//    short sem_op;  // 信号量在一次操作中需要改变的数据，通常是两个数，
// 一个是-1，即P（等待）操作，
// 一个是+1，即V（发送信号）操作。
//   short sem_flg;  // 通常为SEM_UNDO,使操作系统跟踪信号，
//                   // 并在进程没有释放该信号量而终止时，操作系统释放信号量
//};
int P_sem(int semId, int semIndex)
{
    struct sembuf s;
    s.sem_num = semIndex;
    s.sem_op = -1;
    s.sem_flg = SEM_UNDO;

    if(semop(semId,&s,1) < 0)
    {
        printf("op errno,%d: %s\n", errno, strerror(errno));
        return -1;
    }
}

int  V_sem(int semId, int semIndex)
{
    struct sembuf s;
    s.sem_num = semIndex;
    s.sem_op = 1;
    s.sem_flg = SEM_UNDO;
    if(semop(semId,&s,1) < 0)
    {
        printf("ov error,%d:%s\n",errno,strerror(errno));
        return -1;
    }
}

// union semun {
// int val;
// struct semid_ds *buf;
// unsigned short *arry;
// };
// SETVAL：用来把信号量初始化为一个已知的值。
// p 这个值通过union semun中的val成员设置，其作用是在信号量第一次使用前对它进行设置。
// IPC_RMID：用于删除一个已经无需继续使用的信号量标识符。

int DestorySem(int semId)
{
    semun sem_union;

    if (semctl(semId, 0, IPC_RMID, sem_union) == -1)
    {
        printf("err to delete semaphore!\n");

        return -1;
    }
}
/***************************************************************  进程间的信号量通信 ***************************************************/


/***************************************************************  进程间的内存共享通信 ***************************************************/
//key：共享内存名字，确保不同进程看到同一份IPC资源；
//其中key值的又由ftok函数创建。
//size：共享内存的大小，共享内存的创建是以页为单位的;
//页的大小是4096k（4kb）
//共享内存分别存放Odom Imu Gps Robot数据  请按照此顺序进行读取以及写入
int InitShareMemory(int &shmid, int key, int size, int cmd)
{
    int pageSize = getpagesize();//获取系统页面的大小

    if(size <= 0)
    {
        std::cout << "共享内存创建非法，请检查！" << std::endl;
        return -1;
    }
    else
    {
        shmid = shmget(key, size, cmd);
        if(shmid < 0)
        {
            std::cout << "InitShm failed" << std::endl;
            return -1;
        }
        else
        {
            std::cout << "InitShm success" << std::endl;
            return 0;
        }

        return 0;
    }

}

int CreatShareMemory(int &shmid, key_t key, int size, int cmd)
{
    shmid = shmget(key, size, IPC_CREAT | 0666);

    if( shmid < 0)
    {
        perror("CreatShm failed");
        return -1;
    }
    else
    {
        std::cout << "CreatShm successful" << std::endl;
        return 0;
    }
}

void* AttachShareMemory(int shmid)
{
    void* shmaddr;
    return shmaddr = (void *) shmat(shmid, NULL, 0);
}

void* WriteShareMemory(void* shmaddr, void* source, int size)
{
    return memcpy(shmaddr, source, size);
}

void* ReadShareMemory(void* destion, void* shmaddr, int size)
{
    return memcpy(destion, shmaddr, size);
}

int DisattachShareMemory(void* shmaddr)
{
    return shmdt(shmaddr);
}

int DestoryShareMemory(int shmid)
{
    return shmctl(shmid, IPC_RMID, NULL);
}
/***************************************************************  进程间的内存共享通信 ***************************************************/
