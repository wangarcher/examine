#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

#define w 1000
#define D 50.0
#define LANENUM 7


typedef struct selfpoint
{
    int lat;
    int lon;
};

void laneDrawing(Mat img, int count, int lane_num, float dis, std::vector<selfpoint> sequence, std::vector<vector<selfpoint>> &all);
void filledCircle(Mat img, Point center);
void line(Mat img, Point start, Point end);