#include <iostream>
#include <string>
#include "../include/DMzxing.h"

using namespace cv;
using namespace std;
/*
std::string DataMatrixDecoder(std::string image_path) 
{

    //初始化解析结果
    std::string content = "";
    //读取图片
    cv::Mat matSrc = cv::imread(image_path, 1);
    //读取失败则返回
    if (!matSrc.data)
    {
        fprintf(stderr, "read image error: %s", image_path.c_str());
        return content;
    } 
    try
    {
        //转化为灰度图
        cv::Mat matGray;cv::cvtColor(matSrc, matGray, CV_BGR2GRAY); 
        zxing::Ref source = MatSource::create(matGray);
        int width = source->getWidth();
        int height = source->getHeight();
        
        //fprintf(stderr, "image width: %d, height: %d\n", width, height);
        zxing::Ref reader;
        reader.reset(new zxing::datamatrix::DataMatrixReader);
        zxing::Ref binarizer(new zxing::GlobalHistogramBinarizer(source));
        zxing::Ref bitmap(new zxing::BinaryBitmap(binarizer));

        //开始解码
        zxing::Ref result(reader->decode(bitmap, 

        zxing::DecodeHints(zxing::DecodeHints::DATA_MATRIX_HINT)));
        //获取解析结果
        content = result->getText()->getText();
}

catch (zxing::Exception e)

{

}

return content;

}
*/



void dmZxing(cv::Mat frame)
{
    cv::Mat matGray;
    cv::cvtColor(frame, matGray, COLOR_BGR2GRAY);
    try {
        zxing::Ref<zxing::LuminanceSource> source = MatSource::create(matGray);
        int width = source->getWidth();
        int height = source->getHeight();
        fprintf(stderr, "image width: %d, height: %d\n", width, height);

        zxing::Ref<zxing::Reader> reader;
        reader.reset(new zxing::qrcode::QRCodeReader);

        zxing::Ref<zxing::Binarizer> binarizer(new zxing::GlobalHistogramBinarizer(source));
        zxing::Ref<zxing::BinaryBitmap> bitmap(new zxing::BinaryBitmap(binarizer));
        cout << "sxxxxcx" << endl;
        //开始解码
        zxing::Ref<zxing::Result> result(reader->decode(bitmap, zxing::DecodeHints(zxing::DecodeHints::QR_CODE_HINT)));

        std::string str2 = result->getText()->getText();

        cout << str2.data();
        imshow("corrected2",frame);
        cvWaitKey(10);

    }
    catch (zxing::Exception e)
    {
        cout << "fuxking error" << endl;
    }
}

