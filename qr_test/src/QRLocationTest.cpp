#include "../include/QRLocation.h"
#include <cmath>
#include <iostream>
#include <stdio.h>

using namespace std;

float x;
float y;
float yaw;

float qry = -1.5;

int main()
{
    QRLocation qrLoc;
    if(!qrLoc.init(0,0.81,true))
        return 1;
    QRPose_t pose;
    while(true)
    {
        if(qrLoc.getQRPose(&pose))
        {
            double aInDegree=pose.a;
            double bInDegree=pose.b;
            printf("a=%.2lf,b=%.2lf,z=%.2lf\n",aInDegree,bInDegree,pose.z);
            x = tan(aInDegree) * pose.z * 0.01 + pose.qrDist;
            y = pose.z * 0.01 + qry;
            cout  << "x: " << x << " ,y: " << y << endl;   
        }
    }
}
