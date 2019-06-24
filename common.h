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

// ****** settings ******//
#define GALAXY;
// for armor
//#define DEBUG_ARMOR_DETECT
//#define SHOW_PUT_TEXT
//#define SHOW_DRAW
#define USE_FIT_ELLIPSE
// for buff
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

#define DEBUG_BUFF_DETECT

// for image
#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 360
#define BUFFER_SIZE 1

// ****** common ******//
#define END_THREAD if(end_thread_flag) return;
#define INFO(a) cout<<#a<<"="<<a<<endl;
#define TIME_START(a) double a=getTickCount();
#define TIME_END(a) cout<<#a<<" "<<(getTickCount()-a)*1000/getTickFrequency()<<endl;
#define NOTICE(test, num){                   \
    static bool flag = true;            \
    static int i=0; \
    if(flag)                            \
    {              \
        i++;                          \
        std::cout << test << std::endl; \
    if(i>=num)               \
        flag = false;                   \
    }                                   \
}                                       \

//#define TIMER_START boost::timer t_##__func__;
//#define TIMER_END std:: cout << "[" << #__func__ << "]" << "cost time: " << t_##__func__.elapsed() << std::endl;


/**
 * @brief The Param_ class 参数传入传出
 */
class Parameter
{
public:
    Parameter()
    {
        cap_mode_ = 0;
        color = 0;
        buff_offset_x = 121;
        buff_offset_y = 90;
        short_offset_x = 111;
        short_offset_y = 80;
        long_offset_x = 90;//90
        long_offset_y = 200;//100
        world_offset_x = 500;
        world_offset_y = 639;
        offset_image = Point2i(100,100);
        color_th = 65;
        gray_th = 108;
        points_2d.clear();
    }
    int8_t color;
    Point2i offset_image;
    bool cap_mode_;
    // auto aim
    int short_offset_x;
    int short_offset_y;
    int long_offset_x;
    int long_offset_y;
    bool is_small;
    // buff aim
    int buff_offset_x;
    int buff_offset_y;
    int world_offset_x;
    int world_offset_y;
    float buff_angle;

    int color_th;
    int gray_th;

    vector<Point2f> points_2d;
};


