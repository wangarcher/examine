#include "../include/DMzxing.h"
#include "../include/Para.h"
#include "../include/Information_share_out.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdio.h>

using namespace std;
using namespace cv;
int count;
float state_x, state_y, state_yaw, sent_yaw, qrX, qrY, qrT;
int sent_x, sent_y, sent_conf, qrindex;

int main()
{
    Mat frame;
    VideoCapture capture(0);    
    std::string image_name = "../ZK.png";
    cv::Mat matSrc = cv::imread(image_name, 1);
    while(1)
    {
        capture >> frame;
        if (!frame.data)
        {
            cout << "camera fuxked!" << endl;
            return 1;
        }
        dmZxing(frame);
    }
    return 0;
}