#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <cmath>

#include "../include/line_drawing.h"

using namespace cv;
using namespace std;

int latitude, longitude;

int main(void)
{
	int count = 0;
    std::vector<selfpoint> sequence;
	std::vector<selfpoint> sequence_left;
    std::vector<selfpoint> sequence_right;

	ifstream fin("../pointInfo.txt"); 
    while(!fin.eof())
    {
        fin >> latitude >> longitude;
		selfpoint origin;
		origin.lat = latitude;
		origin.lon = longitude;
                sequence.push_back(origin);
		count++;
    }
    char line_window[] = "fxxking line";
    Mat line_image = Mat::zeros( w, w, CV_8UC3 );

	for(int j = 1; j < count-1; j++)
	{
		float x0 = sequence[j-1].lat;
		float y0 = sequence[j-1].lon;
		float x1 = sequence[j].lat;
		float y1 = sequence[j].lon;
		float x2 = sequence[j+1].lat;
		float y2 = sequence[j+1].lon;
		
		float k = -1/((y2 - y0)/(x2 -x0));
		float dx_p = D/(2*sqrt(k*k+1));
		float dx_n = -dx_p;
		float dy_p = dx_p * k;
		float dy_n = dx_n * k;

	
		selfpoint left;
		left.lat = x1 + dx_n;
		left.lon = y1 + dy_n;
		selfpoint right;
		right.lat = x1 + dx_p;
		right.lon = y1 + dy_p;
		
	    sequence_left.push_back(left);
        sequence_right.push_back(right);
	}

    for(int i = 0; i < count; i++)
	{
    	filledCircle(line_image, Point(sequence[i].lat, sequence[i].lon));
		filledCircle(line_image, Point(sequence_left[i].lat, sequence_left[i].lon));
		filledCircle(line_image, Point(sequence_right[i].lat, sequence_right[i].lon));

		if(i+1< count)
		{
			line(line_image, Point(sequence[i].lat, sequence[i].lon), Point(sequence[i+1].lat, sequence[i+1].lon));
		}

	}
	imshow(line_window, line_image);
	moveWindow(line_window, 0, 200);
    waitKey(0);
    return(0);
}

void filledCircle( Mat img, Point center)
{
  circle(img, center, 3, Scalar(125,125,255), FILLED, LINE_8);
}

void line( Mat img, Point start, Point end)
{
  int thickness = 1;
  line(img, start, end, Scalar(0, 255, 0 ), thickness, LINE_8);
}
