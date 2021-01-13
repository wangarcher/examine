// Created by finley on 2020/12/8.
// Copyright (c) 2020  All rights reserved.

//# -*- coding: utf-8 -*-
// @FileName: Para.h
// @Email: Guang.yang@i-usv.com
// @Software: CLion
// @Project: Localization
#ifndef NAVIGATION_PARA_CONFIGURE_H
#define NAVIGATION_PARA_CONFIGURE_H

#include <string>

const std::string QRcodeSemPath = "/Doc/QRcodeSem.txt";
const std::string QRcodeMemPath = "/Doc/QRcodeShm.txt";
const std::string LidarObstacleSemPath = "/Doc/LidarObstacleSem.txt";
const std::string LidarObstacleMemPath = "/Doc/LidarObstacleShm.txt";
const std::string VisualObstacleSemPath = "/Doc/VisualObstacleSem.txt";
const std::string VisualObstacleMemPath = "/Doc/VisualObstacleShm.txt";

const std::string ProgramPath  = "/Localization";


extern std::string PwdQRcodeSemPath;
extern std::string PwdQRcodeMemPath;
extern std::string PwdLidarObstacleSemPath;
extern std::string PwdLidarObstacleMemPath;
extern std::string PwdVisualObstacleSemPath;
extern std::string PwdVisualObstacleMemPath;

void InitPara();

#endif //NAVIGATION_PARA_H