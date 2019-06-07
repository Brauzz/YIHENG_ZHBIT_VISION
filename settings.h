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
        color = 1;
        offset_x = 121;
        offset_y = 90;
        short_offset_x = 121;
        short_offset_y = 90;
        long_offset_x = 100;
        long_offset_x = 100;
        offset_image = Point2i(100,100);
        color_th = 10;
        gray_th = 80;
        points_2d.clear();
    }
    int8_t color;
    Point2i offset_image;
    int short_offset_x;
    int short_offset_y;
    int long_offset_x;
    int long_offset_y;
    int offset_x;
    int offset_y;
    int color_th;
    int gray_th;
    vector<Point2f> points_2d;
};


