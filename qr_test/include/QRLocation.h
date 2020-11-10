#ifndef QRLOCATION_H
#define QRLOCATION_H

/*
二维码的内容必须符合格式：
QRLocation,<qrSize>
其中<qrSize>是一个实数，表示二维码边长
*/

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <zbar.h>

//二维码倾斜阈值
#define QRLOCATION_INCLINATION_THRESHOLD 0.1
//调试窗口标题
#define QRLOCATION_DEBUGUI_TITLE "ssdd"

//二维码位姿
typedef struct QRPose
{
    //二维码中心所在铅垂线与O点构成的平面和Z轴形成的夹角
    float a;
    //二维码所在平面与X轴构成的夹角
    float b;
    //二维码中心到XOY平面的距离
    float z;
    //二维码标识距离
    float qrDist;
}
QRPose_t;

//二维码定位算法
class QRLocation
{

public:
    //初始化，第一个参数为摄像头编号，第二个参数为摄像头上下视角，第三个参数为是否开启调试窗口
    bool init(int webcamIndex, float hViewAngle, bool debugUI);
    //获取二维码位姿
    bool getQRPose(QRPose_t* qrPose);
    //销毁
    bool destroy();

private:
    //摄像头
    CvCapture* capture;
    //摄像头上下视角
    float hViewAngle;
    //是否开启调试窗口
    bool debugUI;
    //灰度图
    IplImage* grayFrame;
    //图片扫描器
    zbar::ImageScanner scanner;

private:
    //计算位姿（格式合法性判断）
    bool getQRPose(zbar::Image::SymbolIterator symbol,QRPose_t* qrPose);
    //计算位姿（算法）
    bool getQRPose(zbar::Image::SymbolIterator symbol,float qrSize, float qrDist, QRPose_t* qrPose);

};

#endif
