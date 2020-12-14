#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>
#include <fstream>
using namespace std;
using namespace cv;
#define PATH "../"
#define NUM 7
int main() {
    Mat image_in; 
    // 定义用来保存文件路径的容器
    vector<string> filelist;
    // 定义用来保存旋转和平移矩阵的容器
    vector<Mat> rvecs, tvecs;
    // 定义相机矩阵，畸变矩阵
    Mat cameraMatrix;
    Mat distCoeffs;
    int flags = 0;
    // 定义保存图像二维角点的容器
    vector<Point2f> corners;
    // 定义保存图像三维角点的容器
    vector<vector<Point2f> > corners2;
    // 定义保存图像二维和三维角点的容器
    vector<Point3f> worldPoints;
    vector<vector<Point3f> > worldPoints2;
//***************读取一个文件夹中的所有图片（所有标定图片）**********************
    for(int i=1; i<NUM;i++) {
        stringstream str;
        str << PATH << i << ".jpg";
        cout << str.str() << endl;
        // 保存所有图片的路径，放入容器filelist中
        filelist.push_back(str.str());
        image_in = imread(str.str());
    }
//***************************找角点××××××××××××××××××××××××××××××××
    for(int i=0;i<filelist.size();i++){
       //cout <<filelist[i]<<endl;
       // 一张张读入图片；
       image_in = imread(filelist[i]);
       // 找图片的角点，参数分别为：
       // 输入图片，图片内角点数（不算棋盘格最外层的角点），输出角点，求解方式
       bool found = findChessboardCorners(image_in, Size(8,6),corners,CALIB_CB_ADAPTIVE_THRESH|CALIB_CB_NORMALIZE_IMAGE);
       // 将找到的角点放入容器中；
       corners2.push_back(corners);
       //画出角点
       drawChessboardCorners(image_in,Size(9,6),corners, found);
       //显示图像
       imshow("test",image_in);
       // 图像刷新等待时间，单位ms
       waitKey(100);
       // 世界坐标系的二维vector 放入三维vector
        worldPoints2.push_back(worldPoints);
    }
//***********************生成一组object_points*************************
    for(int j = 0;j<9;j++){
        for(int k = 0; k<6;k++){
            worldPoints.push_back(Point3f(j*1.0 ,k*1.0 ,0.0f)); 
                worldPoints.clear();
          
        }
    }
    cout << "KKK" << endl;
    calibrateCamera(worldPoints2,corners2,image_in.size(),cameraMatrix,distCoeffs, rvecs,tvecs, CV_CALIB_FIX_PRINCIPAL_POINT);
//*************************************查看参数*****************************************
    cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << endl;
    cout << cameraMatrix.at<double>(0,0) << " " << cameraMatrix.at<double>(0,1) << " " << cameraMatrix.at<double>(0,2) << endl;
    cout << cameraMatrix.at<double>(1,0) << " " << cameraMatrix.at<double>(1,1) << " " << cameraMatrix.at<double>(1,2) << endl;
    cout << cameraMatrix.at<double>(2,0) << " " << cameraMatrix.at<double>(2,1) << " " << cameraMatrix.at<double>(2,2) << endl;
    cout << distCoeffs.rows << "x" <<distCoeffs.cols << endl;
    cout << distCoeffs << endl;
    for(int i = 0;i < distCoeffs.cols;i++)
    {
        cout << distCoeffs.at<double>(0,i) << " " ;
    }
    cout <<endl;
    return 0;
}

