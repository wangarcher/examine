#include <iostream>
#include <cmath>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <pthread.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include "../include/Odom.h"
#include "../include/Imu.h"
#include "../include/Mtime.h"
#include "../include/UartCom.h"

OdomData odom;
OdomData* podom = new OdomData();

int odomfd; //串口标识
pthread_mutex_t odomMutex;

OdomData::OdomData()
{
    this->rightEncDt = 0;
    this->leftEncDt  = 0;
    this->rightDistance = 0;
    this->leftDistance  = 0;
    this->theta = 0;
    this->wSpeed = 0;
    this->vSpeed = 0;
}

OdomData::~OdomData()
{
    delete podom;
}

void OdomData::ClearOdomData()
{
    this->rightEncDt = 0;
    this->leftEncDt  = 0;
    this->rightDistance = 0;
    this->leftDistance  = 0;
    this->theta = 0;
    this->wSpeed = 0;
    this->vSpeed = 0;
}

void OdomData::ParsingOdomData(int rightEncDt_, int leftEncDt_)
{
    this->SetRightAndLeftEncDt(rightEncDt_, leftEncDt_);
    this->SetDistanceRightAndLeft();
    this->SetDistanceAndTheta();
    this->SetSpeed();
    this->OdomDeadReackoning();
}

void OdomData::OdomDeadReackoning()
{
    float delta_x, delta_y, delta_distance,delta_theta;
    float x, y,theta;
    float Pitch, Roll, Yaw;
    odom.GetDistanceAndTheta(delta_distance, delta_theta);
    odom.GetPos(x,y,theta);

  //  imu.GetPostureYPR(Pitch, Roll, Yaw);
  //  Yaw/=57.3;

    delta_x = delta_distance * cos(theta + delta_theta/2.0);
    delta_y = delta_distance * sin(theta + delta_theta/2.0);
    

    //delta_x = delta_distance * cos(theta + Yaw/2.0);
   // delta_y = delta_distance * sin(theta + Yaw/2.0);
    x = x + delta_x;
    y = y + delta_y;
    theta = theta + delta_theta/2.0;
    odom.SetPos(x, y, theta);
}

void* OdomData::ReadOdomData(void* arg)
{
	float x_,y_,theta_;
	//gps.getGpsPosture(x_,y_,theta_);
    //odom.SetPos(x_, y_, theta_);
    while(1)
	{
		unsigned char buf[15];
		union chtosh cts;
		struct can_frame frame;

		int leftCnt, rightCnt;
		int ret = read(odomfd,(char *)&frame,sizeof(frame));
		if( 0x11 != frame.can_id ) 
		{
			continue;
		}
		if((frame.can_dlc == 6)&&(frame.data[0] == 0xaa)&&(frame.data[1] == 0x55))
		{
		    cts.ch[0] = frame.data[2];
		    cts.ch[1] = frame.data[3];
		    leftCnt = cts.sh-128;
		    cts.ch[0] = frame.data[4];
		    cts.ch[1] = frame.data[5];
		    rightCnt = cts.sh-128;
			
		   // odom.ClearOdomData();
            odom.ParsingOdomData(rightCnt, leftCnt);
/*
            int rightEncDt_;
            int leftEncDt_;
            float rightDistance_;
            float leftDistance_;

	        float aveDistance_;
	        float theta_;
	        float vSpeed_;
	        float thetaSpeed_;
            float x_;
            float y_;
            float theta;

		    odom.GetRightAndLeftEncDt(rightEncDt_,leftEncDt_);
		    odom.GetDistanceRightAndLeft(rightDistance_,leftDistance_); 
		    odom.GetDistanceAndTheta(aveDistance_,theta_);
		    odom.GetSpeed(vSpeed_, thetaSpeed_);
		    odom.GetPos(x_, y_, theta);
*/
		}  
		usleep(5000);
	}  
}

int OdomData::InitSocket()
{
	odomfd = socket(PF_CAN,SOCK_RAW,CAN_RAW);
    struct sockaddr_can mycan_addr ;
    struct ifreq myifr;
    struct can_filter canfilter[1];
    strcpy(myifr.ifr_name,"can1");
    ioctl(odomfd,SIOCGIFINDEX,&myifr);
    mycan_addr.can_family  = AF_CAN;
    mycan_addr.can_ifindex = myifr.ifr_ifindex;
    if(bind(odomfd, (struct sockaddr *)&mycan_addr,sizeof(mycan_addr)))
    {
        printf("can bind:error\n");
        return -1;
    }
    canfilter[0].can_id   = 0x11;
    canfilter[0].can_mask = CAN_SFF_MASK;
    setsockopt(odomfd, SOL_CAN_RAW, CAN_RAW_FILTER, &canfilter, sizeof(canfilter));
	
    return 0;
}

int OdomData::StartReadOdomDataThread()
{
    pthread_t ReadOdomThread;
    if(0 != pthread_create(&ReadOdomThread, NULL, ReadOdomData, (void*)this))
    {
        return -1;
    }

    return 0;
}

int OdomData::InitOdom()
{
    //初始化CAN口
    int ret = odom.InitSocket();

    //初始化读写线程
    if(ret == 0)
    {
        ret = odom.StartReadOdomDataThread();
        if(0 != ret)
        {
            std::cout << "Create ReadOdomThread failed!" << std::endl;
            return -1;
        }
    }
    else
    {
        return -1;
    }
    
    return 0;
}

void OdomData::SetRightAndLeftEncDt(int rightEncDt_, int leftEncDt_)
{
    this->rightEncDt = rightEncDt_;
    this->leftEncDt  = leftEncDt_;
}

void OdomData::GetRightAndLeftEncDt(int &rightEncDt_, int &leftEncDt_)
{
    rightEncDt_ = this->rightEncDt;
    leftEncDt_ = this->leftEncDt;
}

void OdomData::SetDistanceRightAndLeft()
{
    this->rightDistance = this->rightEncDt * PI * D * 39 / 49 / 2000 * 10.0 / 9.0;
    this->leftDistance  = this->leftEncDt * PI * D * 39 / 49  / 2000 * 10.0 / 9.0;
}

void OdomData::GetDistanceRightAndLeft(float &rightDistance_, float &leftDistance_)
{
    rightDistance_ = this->rightDistance;
    leftDistance_  = this->leftDistance;
}

void OdomData::SetDistanceAndTheta()
{
    this->aveDistance = (this->rightDistance + this->leftDistance) / 2;
    this->theta = (this->rightDistance - this->leftDistance) / WHEELDIST; //通过弧长公式 θ=l/Rθ=l/R
}

void OdomData::GetDistanceAndTheta(float &aveDistance_, float &theta_)
{
    aveDistance_ = this->aveDistance;
    theta_ = this->theta;
}

void OdomData::SetSpeed()
{
	static long int LastTime = 0;
	long int NowTime=getSysTime();
	if( 0 == LastTime )
	{
		LastTime = NowTime+10;
	}
	long int  DeltaTime = NowTime - LastTime ;
	LastTime = NowTime;
	//std::cout<<"ODOM  DeltaTime="<<DeltaTime<<std::endl;
   // this->vSpeed = this->aveDistance / ( DeltaTime /1000.0);
    //this->wSpeed = this->theta / ( DeltaTime /1000.0);
    this->vSpeed = this->aveDistance / ((float)dt/1000000.0);
    this->wSpeed = this->theta / ((float)dt/1000000.0);
}

void OdomData::GetSpeed(float &vSpeed_, float &wSpeed_)
{
    vSpeed_ = this->vSpeed;
    wSpeed_ = this->wSpeed;
}

void OdomData::SetPos(float x_, float y_, float theta_)
{
    this->odomPosition.x = x_;
    this->odomPosition.y = y_;
    this->odomPosition.theta = theta_;
}

void OdomData::GetPos(float &x_, float &y_, float &theta)
{
    x_ = this->odomPosition.x;
    y_ = this->odomPosition.y;
    theta = this->odomPosition.theta;
}

void OdomData::PrintRightAndLeftEncDt()
{
    int rightEncDt, leftEncDt;
    rightEncDt = 0;
    leftEncDt = 0;
    this->GetRightAndLeftEncDt(rightEncDt, leftEncDt);
    std::cout << "rightEncDt and leftEncDt is: [" << rightEncDt << " " << leftEncDt << "]" << std::endl;
}
