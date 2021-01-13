#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

Point left_line[2];
Point right_line[2];

void process(Mat &frame, Point *left_line, Point *right_line);
Mat fitLines(Mat &image, Point *left_line, Point *right_line);