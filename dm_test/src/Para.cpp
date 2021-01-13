// Created by finley on 2020/12/8.
// Copyright (c) 2020  All rights reserved.

//# -*- coding: utf-8 -*-
// @FileName: Para.cpp
// @Email: Guang.yang@i-usv.com
// @Software: CLion
// @Project: Localization

#include <iostream>
#include <fstream>
#include <string>
#include <limits.h>
#include <stdlib.h>

#include "../include/Para.h"


std::string PwdQRcodeSemPath;
std::string PwdQRcodeMemPath;
std::string PwdLidarObstacleSemPath;
std::string PwdLidarObstacleMemPath;
std::string PwdVisualObstacleSemPath;
std::string PwdVisualObstacleMemPath;

void InitPara()
{
    char buff[256];
    std::string str("./");

    //查找相对路径
    realpath(str.c_str(),buff);
    //std::cout << "path:>" << str << std::endl;
    //std::cout << "pwd:>" << buff << std::endl;

    //方便调试， vs code 和 cmake的相对路径不一致
    std::string pwd = buff;

    int ret =pwd.find(ProgramPath, 0); //第二个pos（position）为可选参数，省略时默认为0。给定pos即从某位置起出现的第一个位置
    //std::cout << pwd.length() << std::endl;
    //std::cout << ProgramPath.length() << std::endl;
    //std::cout << ret << std::endl;
    pwd.erase(ret + ProgramPath.length(), pwd.length()- ProgramPath.length() - ret);
    //std::cout << pwd << std::endl;

    //查找相对路径
    ret = pwd.find(ProgramPath, 0);
    pwd.erase(ret,ret + ProgramPath.length());
    //pwd += IpcPath;
    //std::cout << pwd << std::endl;

    PwdQRcodeSemPath = pwd + QRcodeSemPath;
    PwdQRcodeMemPath = pwd + QRcodeMemPath;

    PwdLidarObstacleSemPath = pwd + LidarObstacleSemPath;
    PwdLidarObstacleMemPath = pwd + LidarObstacleMemPath;

    PwdVisualObstacleSemPath = pwd + VisualObstacleSemPath;
    PwdVisualObstacleMemPath = pwd + VisualObstacleMemPath;


    ///td::cout << PwdFullModePath.c_str() << std::endl;

    std::cout << "PwdQRcodeMemPath:" << PwdQRcodeMemPath << std::endl;
    std::cout << "PwdLidarObstacleMemPath:" << PwdLidarObstacleMemPath<< std::endl;
    std::cout << "PwdVisualObstacleMemPath:" << PwdVisualObstacleMemPath<< std::endl;
}