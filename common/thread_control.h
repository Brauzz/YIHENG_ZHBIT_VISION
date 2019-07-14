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
#include <opencv2/opencv.hpp>
#include <iostream>
#include "thread"
#include "unistd.h"
#include "chrono"
#include "fstream"

#include "../camera/camera_device.h"
#include "../armor_detection/armor_detect.h"
#include "../buff_detection/buff_detect.h"
#include "./serial/serial_port.h"
#include "./solve_angle/solve_angle.h"
#include "./filter/predict.h"

using namespace cv;
using namespace std;

/**
 * @brief 图像信息，用于线程之间的图像传输
 */
struct ImageData
{
    Mat img;
    float gimbal_data;
    bool serial_success = 1;
    //    unsigned int frame;
};


struct OtherParam
{
    int8_t color = 0;       // 我方车辆颜色，0是蓝色，1是红色。用于图像预处理
    int8_t mode = 0;        // 视觉模式，0是自瞄模式，1是能量机关模式
    int8_t cap_mode = 1;    // 摄像头类型，0是短焦摄像头，1是长焦摄像头
};

// ****** systems  ******//
#define SHOT_CAMERA_THREAD
#define LONG_CAMERA_THREAD
#define PROCESS_IMAGE_THREAD
#define GET_STM32_THREAD
//#define GET_GIMBAL_THREAD
//#define WAITKEY
//#define IMAGESHOW
// ****** settings ******//
#define GALAXY;
// for armor --------------
//#define DEBUG_ARMOR_DETECT
//#define DEBUG_BUFF_DETECT
//#define SHOW_PUT_TEXT
#define SHOW_DRAW
#define USE_FIT_ELLIPSE
//#define PREDICT
// for buff --------------
//#define DEBUG_BUFF_DETECT

// for image
#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 360
#define BUFFER_SIZE 1




void protectDate(int& a, int &b, int &c, int& d, int& e, int& f);
void limit_angle(float &a, float max);

/**
 * @brief 线程管理类
 * 负责图像生成、图像处理、串口数据接收
 */
class ThreadControl
{
public:
    ThreadControl();          // 线程管理构造函数，用于线程中变量初始化
    void ImageProduce();      // 短焦摄像头获取图像线程
    void ImageProduceLong();  // 长焦摄像头获取图像线程
    void ImageProcess();      // 图像处理线程，用于自瞄，能量机关识别
    void GetGimbal();         // 用于获取云台陀螺仪数据
    void GetSTM32();          // 用于接收电控发来的数据

private:
//    SerialPort serial_;
    ImageData data[BUFFER_SIZE];
    OtherParam other_param;
    bool end_thread_flag = false;
//    unsigned int produce_index;
//    unsigned int consumption_index;
//    unsigned int gimbal_data_index;
};

class GimbalDataProcess
{
public:
    GimbalDataProcess():
        count (0),
        last_gimbal_yaw(0){}

    // 陀螺仪角度(-180~180)数据处理 ->(-99999~99999)
    void ProcessGimbalData(float raw_yaw, float &dst_yaw)
    {
        if(raw_yaw - last_gimbal_yaw > 270)
        {
            count --;
        }else if(raw_yaw - last_gimbal_yaw < -270)
        {
            count++;
        }
        dst_yaw= count*360+raw_yaw;
        last_gimbal_yaw = raw_yaw;
    }
private:
    int count;
    float last_gimbal_yaw;
};


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



//                printf("debug test: armor_type = %d, bullet_speed = %f", final_armor_type, bullet_speed_);
//                putText(image, "origin z :" + to_string(distance) + " x: " + to_string(angle_x) + " y :" + to_string(angle_y) + "theta_y :" + to_string(theta_y)
//                        , Point(0,20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
//                putText(image, "orICRA z :" + to_string(distance_i) + " x: " + to_string(angle_x_i) + " y :" + to_string(angle_y_i)
//                        , Point(0,40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));



//    ofstream file("test.txt");
//        file << csmIdx << " " << angle_x << " " << angle_y << "\n";
//    file.close();
