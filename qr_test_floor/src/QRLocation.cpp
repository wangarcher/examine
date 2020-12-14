#include "../include/QRLocation.h"
#include <string.h>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace zbar;
using namespace cv;

float qrSize = 0.174;
//Mat cameraMatrix = (cv::Mat_<float>(3, 3) << 584.316, 0, 0, 2.0289, 584.500,0,  0, 0, 1);;
//Mat distCoeffs = (cv::Mat_<float>(1, 5) <<  -0.3731, 0.256, 0.256, 0.0008, 0.0014);

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
    //img = cvarrToMat(frame);
    //Mat show;
    //undistort(img, show, cameraMatrix, distCoeffs);
    //IplImage pBinary = IplImage(show);
    //grayFrame2 = cvCloneImage(&pBinary);
    if(debugUI)
    {
        cvShowImage(QRLOCATION_DEBUGUI_TITLE,grayFrame);
        //cvShowImage("final input", grayFrame2);
        cvWaitKey(5);
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
    cout << "Point A: " << x0 << ", " << y0 << "\n"
         << "Point B: " << x1 << ", " << y1 << "\n"
         << "Point C: " << x2 << ", " << y2 << "\n"
         << "Point D: " << x3 << ", " << y3 <<  endl;

    //left height of the qrcode
    float yB_A=y1-y0;
    //right height of the qrcode
    float yC_D=y2-y3;
    //make sure the qrcode was well placed

    //left width of the qrcode
    float xB_A=x1-x0;
    //right width of the qrcode
    float xC_D=x2-x3;
    //make sure the qrcode was well placed

    //half view angle tangent value
    float tanw = tan(wViewAngle/2);
    float tanh = tan(hViewAngle/2);

    float k1 = (y2-y0)/(x2-x0);
    float b1 = y0 - k1*x0;
    float k2 = (y3-y1)/(x3-x1);
    float b2 = y1 - k2*x1;
    float crossX = (b1-b2)/(k2-k1);
    float crossY = crossX * k2 + b2;
    float centreX = grayFrame->width/2;
    float centreY = grayFrame->height/2;

    float relateY = (crossX - centreX) / grayFrame->width * tanw * QRLOCATION_CAM_HEIGHT  * 2;
    float relateX = (crossY - centreY) / grayFrame->height * tanh * QRLOCATION_CAM_HEIGHT * 2;
    float leftT, rightT, relateT;
    if(yB_A >= 0)
    {
        if(xB_A > 0)
        {
            leftT = atan(yB_A/xB_A) - 1.57;
        }
        if(xB_A < 0)
        {
            leftT = atan(yB_A/xB_A) + 1.57;
        }
        if(xB_A == 0)
        {
            leftT = 0;
        }
    }
    else    
    {
        if(xB_A > 0)
        {
            leftT = atan(yB_A/xB_A) - 1.57;
        }
        if(xB_A < 0)
        {
            leftT = atan(yB_A/xB_A) + 1.57;
        }
        if(xB_A == 0)
        {
            leftT = -3.14;
        }
    }
    if(yC_D >= 0)
    {
        if(xC_D > 0)
        {
            rightT = atan(yC_D/xC_D) - 1.57;
        }
        if(xC_D < 0)
        {
            rightT = atan(yC_D/xC_D) + 1.57;
        }
        if(xC_D == 0)
        {
            rightT = 0;
        }
    }
    else
    {
        if(xC_D > 0)
        {
            rightT = atan(yC_D/xC_D) - 1.57;
        }
        if(xC_D < 0)
        {
            rightT = atan(yC_D/xC_D) + 1.57;
        }
        if(xC_D == 0)
        {
            rightT = -3.14;
        }
    }
    cout << "leftT: "<< leftT <<  ", rightT: " << rightT << endl;
    if(abs(leftT-rightT) > 1)
    {
        relateT = -abs(leftT);
    }
    else
    {
        relateT = (leftT + rightT) / 2;
    }
    qrPose->relateX = relateX*cos(relateT) - relateY*sin(relateT);
    qrPose->relateY = relateX*sin(relateT) + relateY*cos(relateT);  
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