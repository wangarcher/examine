#pragma once
#ifndef _ODOM_H
#define _ODOM_H

#define dt 10000

#define PI 3.1415926

#define REDUCTIONRATIO 39/49

#define D         0.385
#define WHEELDIST 0.80

typedef struct 
{
    float x;
    float y;
    float theta;
}OdomPosition;

class OdomData
{
private:

    int rightEncDt;
    int leftEncDt;

    float rightDistance;
    float leftDistance;

    float aveDistance;
    float theta;

    float vSpeed;
    float wSpeed;

    OdomPosition odomPosition; //ODOM编码器航迹推演用

private:
    static void* ReadOdomData(void* arg);

public:
    OdomData();
    ~OdomData();
    

    int  InitOdom();

    int  InitSocket();

    int  StartReadOdomDataThread();
    
    void ClearOdomData();

    void ParsingOdomData(int rightEncDt_, int leftEncDt_);

    void OdomDeadReackoning();

    void SetRightAndLeftEncDt(int rightEncDt_, int leftEncDt_);

    void SetDistanceRightAndLeft();

    void SetDistanceAndTheta();

    void SetSpeed();

    void SetPos(float x_, float y_, float theta_);

    void GetRightAndLeftEncDt(int &rightEncDt_, int &leftEncDt_);

    void GetDistanceRightAndLeft(float &rightDistance_, float &leftDistance_); 

    void GetDistanceAndTheta(float &aveDistance_, float &theta_);

    void GetSpeed(float &vSpeed_, float &thetaSpeed_);

    void GetPos(float &x_, float &y_, float &theta);

    void PrintRightAndLeftEncDt();

    //void base_odom()

};

extern OdomData odom;
extern OdomData* podom;

#endif
