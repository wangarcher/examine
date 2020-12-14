#ifndef QRLOCATION_H
#define QRLOCATION_H

/*
qrcode information form;
ZK<X>,<Y>,<yaw>
*/

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>

//tilt threshold 
#define QRLOCATION_INCLINATION_THRESHOLD 0.1
//debugui title set
#define QRLOCATION_DEBUGUI_TITLE "S.S.D.D"
//camera height 
#define QRLOCATION_CAM_HEIGHT 0.15


//qrPose struct
typedef struct QRPose
{
    float relateX;
    float relateY;
    float relateT;
    float qrX;
    float qrY;
    float qrT;
}
QRPose_t;

//qrlocation algorithm
class QRLocation
{

public:
    //initialization video device index, viewangle, debugUI
    bool init(int webcamIndex, float hViewAngle, float wViewAngle, bool debugUI);
    //get pose
    bool getQRPose(QRPose_t* qrPose);

    bool destroy();

private:
    //camera
    CvCapture* capture;
    //viewAngle
    float hViewAngle;
    float wViewAngle;

    bool debugUI;
    //gray frame
    IplImage* grayFrame;
    IplImage* grayFrame2;
    cv::Mat img;
    //img scanner
    zbar::ImageScanner scanner;

private:
    //validity 
    bool getQRPose(zbar::Image::SymbolIterator symbol, QRPose_t* qrPose);
    //pose calculation 
    bool getQRPose(zbar::Image::SymbolIterator symbol,float qrSize, QRPose_t* qrPose);

};

#endif
