#include <opencv2/opencv.hpp>
#include <iostream>
#include <time.h>
using namespace cv;
using namespace std;


Mat cameraMatrix = (cv::Mat_<float>(3, 3) << 640, 0, 640, 0, 640, 480, 0, 0, 1);;
Mat distCoeffs = (cv::Mat_<float>(1, 5) << -0.114339, 0.00056405, 0.00362682, 0.000740103, -0.00701824);
int main()
{
 
    VideoCapture capture(0);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    capture.set(CV_CAP_PROP_FPS, 30);
    capture.set(CV_CAP_PROP_EXPOSURE, 100);
 
    //capture.set(CV_CAP_PROP_BRIGHTNESS, 1);//亮度 50
    capture.set(CV_CAP_PROP_CONTRAST, 5);//对比度 50
    //capture.set(CV_CAP_PROP_SATURATION, 50);//饱和度 50
    //capture.set(CV_CAP_PROP_HUE, 50);//色调 0
    //capture.set(CV_CAP_PROP_EXPOSURE, 50);//曝光 -12
    //打印摄像头参数
    printf("width = %.2f\n", capture.get(CV_CAP_PROP_FRAME_WIDTH));
    printf("height = %.2f\n", capture.get(CV_CAP_PROP_FRAME_HEIGHT));
    printf("fbs = %.2f\n", capture.get(CV_CAP_PROP_FPS));
    printf("brightness = %.2f\n", capture.get(CV_CAP_PROP_BRIGHTNESS));
    printf("contrast = %.2f\n", capture.get(CV_CAP_PROP_CONTRAST));
    printf("saturation = %.2f\n", capture.get(CV_CAP_PROP_SATURATION));
    printf("hue = %.2f\n", capture.get(CV_CAP_PROP_HUE));
    printf("exposure = %.2f\n", capture.get(CV_CAP_PROP_EXPOSURE));
    while (1)
    {
        Mat frame;
        Mat fz;
        Mat show;
        capture >> frame;
        imshow("raw", frame);
        cvWaitKey(1);
    }
    return 0;
}
