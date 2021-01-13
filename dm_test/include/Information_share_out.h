// Created by finley on 2020/12/8.
// Copyright (c) 2020  All rights reserved.

//# -*- coding: utf-8 -*-
// @FileName: Information_share_out.h
// @Email: Guang.yang@i-usv.com
// @Software: CLion
// @Project: Localization
#ifndef NAVIGATION_INFORMATION_SHARE_OUT_H
#define NAVIGATION_INFORMATION_SHARE_OUT_H


#include <iostream>
#include <string>

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

int InitShareMemory(int &shmid, int key, int size, int cmd);
int CreatShareMemory(int &shmid, key_t key, int size, int cmd);
void* AttachShareMemory(int shmid);
void* WriteShareMemory(void* shmaddr, void* source, int size);
void* ReadShareMemory(void* destion, void* shmaddr, int size);
int DisattachShareMemory(void* shmaddr);
int DestoryShareMemory(int shmid);


/*************** 二维码定位模块的信号量 ***********************/
extern int QRcodePosSemKey;
extern int QRcodePosSemId;
/*************** 二维码定位模块的信号量 ***********************/

/*************** 二维码定位模块的共享内存通信 ***********************/
extern int QRcodePosShmKey;
extern int QRcodePosShmId;
/*************** 二维码定位模块的共享内存通信 ***********************/


/*************** 雷达障碍物的信号量 ***********************/
extern int LidarObstacleSemKey;
extern int LidarObstacleSemId;
/*************** 雷达障碍物的信号量 ***********************/
/*************** 雷达障碍物的共享内存通信 ***********************/
extern int LidarObstacleShmKey;
extern int LidarObstacleShmId;
/*************** 雷达障碍物的共享内存通信 ***********************/

/*************** 视觉障碍物的信号量 ***********************/
extern int VisualObstacleSemKey;
extern int VisualObstacleSemId;
/*************** 视觉障碍物的信号量 ***********************/
/*************** 视觉障碍物的共享内存通信 ***********************/
extern int VisualObstacleShmKey;
extern int VisualObstacleShmId;
/*************** 视觉障碍物的共享内存通信 ***********************/
#endif //NAVIGATION_INFORMATION_SHARE_OUT_H
