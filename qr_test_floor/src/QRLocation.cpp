#include "../include/QRLocation.h"
#include <string.h>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace zbar;

float qrSize = 0.174;


bool QRLocation::init(int webcamIndex, float hViewAngle, float wViewAngle, bool debugUI)
{
    capture=cvCreateCameraCapture(webcamIndex);
    if(!capture)
        return false;
    this->hViewAngle = hViewAngle;
    this->wViewAngle = wViewAngle;
    this->debugUI = debugUI;
    grayFrame=0;
    //zbar configuration
    scanner.set_config(zbar::ZBAR_NONE,zbar::ZBAR_CFG_ENABLE,1);

    if(debugUI)
        cvNamedWindow(QRLOCATION_DEBUGUI_TITLE,CV_WINDOW_AUTOSIZE);
}

bool QRLocation::getQRPose(QRPose_t* qrPose)
{
    //get the img from the video stream
    IplImage* frame=cvQueryFrame(capture);
    if(!frame)
        return false;
    if(!grayFrame)
        grayFrame=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U,1);
    cvCvtColor(frame,grayFrame,CV_BGR2GRAY);
    if(debugUI)
    {
        cvShowImage(QRLOCATION_DEBUGUI_TITLE,grayFrame);
        cvWaitKey(50);
    }
    //create zbar img
    Image image(frame->width,frame->height,"Y800",grayFrame->imageData,frame->width*frame->height);
    int symbolCount=scanner.scan(image);
    //get the first qrcode
    Image::SymbolIterator symbol=image.symbol_begin();
    while(symbolCount--)
    {
        //if it was recognized
        if(getQRPose(symbol,qrPose))
            return true;
        //the next qrcode
        ++symbol;
    }
    return false;
}

bool QRLocation::getQRPose(Image::SymbolIterator symbol,QRPose_t* qrPose)
{
    //qrcode integrity check
    if(symbol->get_type_name()!="QR-Code")
        return false;
    //get the information
    char data[128];
    strncpy(data,symbol->get_data().c_str(),sizeof(data)-1);
    data[sizeof(data)-1]=0;
    //the information has to be start with a certainly determined character sequence like "ZK"
    if(strncmp(data,"ZK",2)!=0)
        return false;
    //vital information
    float qrX = 0;
    float qrY = 0;
    float qrT = 0;
    sscanf(data+2, "%f", &qrX);
    sscanf(data+7, "%f", &qrY);
    sscanf(data+12, "%f", &qrT);
    qrPose->qrX = qrX;
    qrPose->qrY = qrY;
    qrPose->qrT = qrT;
    //pose calculation process
    return getQRPose(symbol, qrSize, qrPose);
}

bool QRLocation::getQRPose(Image::SymbolIterator symbol, float qrSize, QRPose_t* qrPose)
{
    //get four angular points of the qrcode
    float x0=symbol->get_location_x(0);
    float y0=symbol->get_location_y(0);
    float x1=symbol->get_location_x(1);
    float y1=symbol->get_location_y(1);
    float x2=symbol->get_location_x(2);
    float y2=symbol->get_location_y(2);
    float x3=symbol->get_location_x(3);
    float y3=symbol->get_location_y(3);

    //left height of the qrcode
    float leftH=y1-y0;
    //right height of the qrcode
    float rightH=y2-y3;
    //make sure the qrcode was well placed

    //left width of the qrcode
    float leftW=x1-x0;
    //right width of the qrcode
    float rightW=x2-x3;
    //make sure the qrcode was well placed

    //half view angle tangent value
    float tanw = tan(wViewAngle/2);
    float tanh = tan(hViewAngle/2);

    float k1 = (y2-y0)/(x2-x0);
    float b1 = (x2*y0-x0*y2)/(x2-x0);
    float k2 = (y3-y1)/(x3-x1);
    float b2 = (x3*y1-x1*y3)/(x3-x1);
    float crossX = -(b1-b2)/(k1-k2);
    float crossY = crossX * k2 + b2;
    float centreX = grayFrame->width/2;
    float centreY = grayFrame->height/2;

    float relateY = (crossX - centreX) / grayFrame->width * tanw * QRLOCATION_CAM_HEIGHT  * 2;
    float relateX = (crossY - centreY) / grayFrame->height * tanh * QRLOCATION_CAM_HEIGHT * 2;
    float leftT, rightT;
    if(atan(leftH/leftW) > 0)
    {
        leftT = atan(leftH/leftW) - 1.57;
    }
    else
    {
        leftT = atan(leftH/leftW) + 1.57;
    }
    if(atan(rightH/rightW) > 0)
    {
        rightT = atan(rightH/rightW) - 1.57;
    }
    else
    {
        rightT = atan(rightH/rightW) + 1.57;
    }
    cout << "leftT: "<< leftT <<  ", rightT: " << rightT << endl;
    float relateT = (leftT + rightT) / 2;
    qrPose->relateX = relateX;
    qrPose->relateY = relateY;
    qrPose->relateT = relateT;

    return true;
}

bool QRLocation::destroy()
{
    //relase the grayframe 
    cvReleaseImage(&grayFrame);
    //destroy the window  
    cvDestroyWindow(QRLOCATION_DEBUGUI_TITLE);
    //relase the memory
    cvReleaseCapture(&capture);
}