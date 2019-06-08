/****************************************************************************
 *  Copyright (C) 2019 cz.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#pragma once

#include "opencv2/opencv.hpp"
#include "iostream"

using namespace cv;
using namespace std;

/**
 * @brief The Param_ class 参数传入传出
 */
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


