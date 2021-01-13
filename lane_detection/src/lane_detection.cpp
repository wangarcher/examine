#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include "../include/lane_detection.h"

using namespace cv;
using namespace std;

/**
**1. video input  
**2. binary
**3. contour
**4. analyse
**5. fitting
**6. drawline
**
*/
void process(Mat &frame, Point *left_line, Point *right_line )
{
	Mat gray,binary;
	//graying
	cvtColor(frame, gray, COLOR_BGR2GRAY);
	
	//threshold(gray, binary, );
	//edge detection
	Canny(gray, binary, 150, 300);
	//imshow("Canny", binary);
	for (size_t i = 0; i < (gray.rows/2+40); i++)
	{
		for (size_t j = 0; j < gray.cols; j++)
		{
			binary.at<uchar>(i, j) = 0;
		}
	}
	imshow("binary", binary);
	
	//find the contour
	vector<vector<Point>> contours;
	findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	Mat out_image = Mat::zeros(gray.size(), gray.type());

	for (int i = 0; i < contours.size(); i++)
	{
		//calcu the area and perimetel
		double length = arcLength(contours[i], true);
		double area = contourArea(contours[i]);
		//cout << "perimetel length:" << length << endl;
		//cout << "area:" << area << endl;
		//outer rectangle enclosure
		Rect rect = boundingRect(contours[i]);
		int h = gray.rows - 50;

		//coutour analyse
		if (length < 5.0 || area < 10.0)
		{
			continue;
		}
		if (rect.y > h)
		{
			continue;
		}

		//rectangle
		RotatedRect rrt = minAreaRect(contours[i]);
		double angle = abs(rrt.angle);
		
		//angle < 50.0 || angle>89.0

		if (angle < 20.0 || angle>84.0) {

			continue;

		}
		

		if (contours[i].size() > 5) {
			RotatedRect errt = fitEllipse(contours[i]);
			if ((errt.angle<5.0) || (errt.angle>160.0))
			{
				if (80.0 < errt.angle && errt.angle < 100.0) {
					continue;
				}
				
			}
		}


		//cout << "开始绘制：" << endl;
		drawContours(out_image, contours, i, Scalar(255), 2, 8);
		imshow("out_image", out_image);

	}
	Mat result = fitLines(out_image, left_line, right_line);
	imshow("result", result);

	Mat dst;
	addWeighted(frame, 0.8, result, 0.5,0, dst);
	imshow("lane-lines", dst);

}

//直线拟合
Mat fitLines(Mat &image, Point *left_line, Point *right_line) {
	int height = image.rows;
	int width = image.cols;

	Mat out = Mat::zeros(image.size(), CV_8UC3);

	int cx = width / 2;
	int cy = height / 2;

	vector<Point> left_pts;
	vector<Point> right_pts;
	Vec4f left;
	

	for (int i = 100; i < (cx-10); i++)
	{
		for (int j = cy; j < height; j++)
		{
			int pv = image.at<uchar>(j, i);
			if (pv == 255) 
			{
				left_pts.push_back(Point(i, j));
			}
		}
	}

	for (int i = cx; i < (width-20); i++)
	{
		for (int j = cy; j < height; j++)
		{
			int pv = image.at<uchar>(j, i);
			if (pv == 255)
			{
				right_pts.push_back(Point(i, j));
			}
		}
	}

	if (left_pts.size() > 2)
	{
		fitLine(left_pts, left, DIST_L1, 0, 0.01, 0.01);
		
		double k1 = left[1] / left[0];
		double step = left[3] - k1 * left[2];

		int x1 = int((height - step) / k1);
		int y2 = int((cx - 25)*k1 + step);

		Point left_spot_1 = Point(x1, height);
		Point left_spot_end = Point((cx - 25), y2);
		

		line(out, left_spot_1, left_spot_end, Scalar(0, 0, 255), 8, 8, 0);
		left_line[0] = left_spot_1;
		left_line[1] = left_spot_end;

	}
	else
	{
		line(out, left_line[0], left_line[1], Scalar(0, 0, 255), 8, 8, 0);
	}




	if (right_pts.size()>2)
	{
		
		Point spot_1 = right_pts[0];
		Point spot_end = right_pts[right_pts.size()-1];

		int x1 = spot_1.x;
		
		int y1 = spot_1.y;

		int x2 = spot_end.x;
		int y2 = spot_end.y;

	

		line(out, spot_1, spot_end, Scalar(0, 0, 255), 8, 8, 0);
		right_line[0] = spot_1;
		right_line[1] = spot_end;

	}
	else
	{
		line(out, right_line[0], right_line[1], Scalar(0, 0, 255), 8, 8, 0);
	}

	return out;
}


int main(int argc, char** argv) {
	//video in
	VideoCapture capture(0);

	int height = capture.get(CAP_PROP_FRAME_HEIGHT);
	int width = capture.get(CAP_PROP_FRAME_WIDTH);
	int count = capture.get(CAP_PROP_FRAME_COUNT);
	int fps = capture.get(CAP_PROP_FPS);
	//initialization

	left_line[0] = Point(0,0);
	left_line[1] = Point(0, 0);
	right_line[0] = Point(0, 0);
	right_line[1] = Point(0, 0);
	cout << height<<"       "<< width<< "       " <<count<< "       " <<fps << endl;

	//get the frame 
	Mat frame;
	while (true) {
		int ret = capture.read(frame);
		if (!ret) {
			break;
		}
		imshow("input", frame);
		process(frame, left_line, right_line);

		char c = waitKey(5);
		if (c == 27) {
			break;
		}	
	}
}