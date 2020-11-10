#include "../include/QRLocation.h"
#include <string.h>
#include <stdio.h>

using namespace std;
using namespace zbar;

bool QRLocation::init(int webcamIndex, float hViewAngle,bool debugUI)
{
    //打开摄像头
    capture=cvCreateCameraCapture(webcamIndex);
    //摄像头不存在
    if(!capture)
        return false;
    this->hViewAngle=hViewAngle;
    this->debugUI=debugUI;
    grayFrame=0;
    //配置zbar图片扫描器
    scanner.set_config(zbar::ZBAR_NONE,zbar::ZBAR_CFG_ENABLE,1);
    //如果开启调试，则创建窗口，名称为“debugui”，自动调整大小
    if(debugUI)
        cvNamedWindow(QRLOCATION_DEBUGUI_TITLE,CV_WINDOW_AUTOSIZE);
}

bool QRLocation::getQRPose(QRPose_t* qrPose)
{
    //从摄像头中抓取一帧
    IplImage* frame=cvQueryFrame(capture);
    //图像为空
    if(!frame)
        return false;
    //如果灰度图没有创建，就创建一个和原图一样大小的灰度图（8位色深，单通道）
    if(!grayFrame)
        grayFrame=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U,1);
    //原图转灰度图
    cvCvtColor(frame,grayFrame,CV_BGR2GRAY);
    //如果开启调试，则显示灰度图
    if(debugUI)
    {
        cvShowImage(QRLOCATION_DEBUGUI_TITLE,grayFrame);
        cvWaitKey(50);
    }
    //创建zbar图像
    Image image(frame->width,frame->height,"Y800",grayFrame->imageData,frame->width*frame->height);
    //扫描图像，识别二维码，获取个数
    int symbolCount=scanner.scan(image);
    //获取第一个二维码
    Image::SymbolIterator symbol=image.symbol_begin();
    //遍历所有识别出来的二维码
    while(symbolCount--)
    {
        //能够识别
        if(getQRPose(symbol,qrPose))
            return true;
        //下一个二维码
        ++symbol;
    }
    return false;
}

bool QRLocation::getQRPose(Image::SymbolIterator symbol,QRPose_t* qrPose)
{
    //首先得是一个二维码
    if(symbol->get_type_name()!="QR-Code")
        return false;
    //获取内容
    char data[128];
    strncpy(data,symbol->get_data().c_str(),sizeof(data)-1);
    data[sizeof(data)-1]=0;
    //内容得是以“QRLocation,”开头
    if(strncmp(data,"QRLocation,",11)!=0)
        return false;
    //获取二维码边长
    float qrSize = 0;
    float qrDist = 0;
    sscanf(data+11, "%f", &qrSize);
    sscanf(data+15, "%f", &qrDist);
    if(qrSize==0)
        return false;
    qrSize = 17.4;
    //计算位姿
    return getQRPose(symbol,qrSize, qrDist, qrPose);
}

bool QRLocation::getQRPose(Image::SymbolIterator symbol,float qrSize, float qrDist, QRPose_t* qrPose)
{
    //获得四个点的坐标
    float x0=symbol->get_location_x(0);
    float y0=symbol->get_location_y(0);
    float x1=symbol->get_location_x(1);
    float y1=symbol->get_location_y(1);
    float x2=symbol->get_location_x(2);
    float y2=symbol->get_location_y(2);
    float x3=symbol->get_location_x(3);
    float y3=symbol->get_location_y(3);
    //左边沿纵向差
    float leftH=y1-y0;
    //右边沿纵向差
    float rightH=y2-y3;
    //必须保证0点高于1点，3点高于2点
    if(leftH<0||rightH<0)
        return false;
    //左边沿横向差
    float leftW=abs(x0-x1);
    //右边沿横向差
    float rightW=abs(x2-x3);
    //不能太倾斜
    if(max(leftW/leftH,rightW/rightH)>QRLOCATION_INCLINATION_THRESHOLD)
        return false;
    //上下视角一半的正切值，因为一直要用，所以先计算出来
    float tanHalfView=tan(hViewAngle/2);
    float leftLen=sqrt(leftH*leftH+leftW*leftW);
    float rightLen=sqrt(rightH*rightH+rightW*rightW);
    //左边沿的深度
    float leftZ=grayFrame->height*qrSize/tanHalfView/2/leftLen;
    //右边沿的深度
    float rightZ=grayFrame->height*qrSize/tanHalfView/2/rightLen;
    //得到中心点的深度
    float z=(leftZ+rightZ)/2;
    //计算b的正弦值
    float sinB=(leftZ-rightZ)/qrSize;
    if(sinB>1)
        return false;
    //得到b
    float b=asin(sinB);
    //两条对角线的系数和偏移
    float k1=(y2-y0)/(x2-x0);
    float b1=(x2*y0-x0*y2)/(x2-x0);
    float k2=(y3-y1)/(x3-x1);
    float b2=(x3*y1-x1*y3)/(x3-x1);
    //两条对角线交点的X坐标
    float crossX=-(b1-b2)/(k1-k2);
    //计算a的正切值
    float tanA=tanHalfView*(2*crossX-grayFrame->width)/grayFrame->height;
    //得到a
    float a=atan(tanA);
    qrPose->a=a;
    qrPose->b=b;
    qrPose->z=z;
    qrPose->qrDist = qrDist;
    return true;
}

bool QRLocation::destroy()
{
    //释放灰度图
    cvReleaseImage(&grayFrame);
    //销毁窗口
    cvDestroyWindow(QRLOCATION_DEBUGUI_TITLE);
    //释放内存
    cvReleaseCapture(&capture);
}
