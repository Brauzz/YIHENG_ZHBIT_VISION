#pragma once

#include "opencv2/opencv.hpp"
#include "iostream"

using namespace cv;
using namespace std;

class Param_
{
public:
    Param_()
    {
        color = 0;
        offset_x = 121;
        offset_y = 100;
        offset_image = Point2f(100,100);
        color_th = 10;
        gray_th = 50;
        points_2d.clear();
    }
    int8_t color;
    Point2f offset_image;
    int offset_x;
    int offset_y;
    int color_th;
    int gray_th;
    vector<Point2f> points_2d;
};


