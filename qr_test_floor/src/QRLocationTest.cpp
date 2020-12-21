#include "../include/QRLocation.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdio.h>

using namespace std;

float state_x, state_y, state_yaw; 

int main()
{

    QRLocation qrLoc;
    if(!qrLoc.init(0, 0.82, 1.04, true))
        return 1;
    QRPose_t pose;
    while(true)
    {
        if(qrLoc.getQRPose(&pose))
        {
            cout << "qrX: "<< pose.qrX << ", qrY: " << pose.qrY << ", qrT: " << pose.qrT << "\n"
                 << "relateX: " << pose.relateX << ", relateY: " << pose.relateY << ", relateT: "<< pose.relateT << endl;
            state_x = pose.qrX + pose.relateX;
            state_y = pose.qrY + pose.relateY;
            state_yaw = pose.qrT + pose.relateT;
            cout << "X: " << state_x << ", Y: " << state_y << ", yaw: " << state_yaw << endl;
        }
    }
}
