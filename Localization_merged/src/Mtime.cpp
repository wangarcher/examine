#include <stdio.h>
#include <iostream>
#include <string>
#include <string.h>
#include <stdlib.h>

#include<time.h>
#include<sys/time.h>

#include "../include/Mtime.h"


void printCurrentTime()
{
    struct tm       *p;
    p = localtime(NULL);

    std::cout << 1900+p->tm_year << "/" << 1 + p->tm_mon << "/" << p->tm_mday; 
    std::cout << " " << p->tm_hour << ":" << p->tm_min << ":" << p->tm_sec; 
}

long int getSysTime()
{
    struct timeval  tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*1000 + tv.tv_usec/1000;
}

int getCurrentTime(char *timedata)
{
    struct timeval  tv;
    struct timezone tz;
    struct tm       *p;
    char ctime[10];

    gettimeofday(&tv, &tz);
    p = localtime(&tv.tv_sec);

    sprintf(ctime, "%d", 1900+p->tm_year); 
	strcpy(timedata,ctime);
    sprintf(ctime, "%d", 1 + p->tm_mon); 
	strcat(timedata,ctime);
    sprintf(ctime, "%d", p->tm_mday); 
	strcat(timedata,ctime);
    sprintf(ctime, "%d", p->tm_hour); 
	strcat(timedata,ctime);
    sprintf(ctime, "%d", p->tm_min); 
	strcat(timedata,ctime);
    sprintf(ctime, "%d", p->tm_sec); 
	strcat(timedata,ctime);
    sprintf(ctime, ".%ld", tv.tv_usec); 
	strcat(timedata,ctime);


    return 0;
}



//获取当前时间   返回值   微秒数
unsigned long GetNowTimeUsec(void)
{
    struct timeval  tv;

    gettimeofday(&tv, NULL);

    return tv.tv_sec * 1000000 + tv.tv_usec;
}

