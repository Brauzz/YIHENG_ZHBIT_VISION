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
#include "../camera/save_video.h"
#include "../armor_detection/armor_detect.h"
#include "../buff_detection/buff_detect.h"
#include "./serial/serial_port.h"
#include "./solve_angle/solve_angle.h"
#include "./filter/predict.h"
#include "../base.h"
#include "../mainwindow.h"
using namespace cv;
using namespace std;

/**
 * @brief 图像信息，用于线程之间的图像传输
 */


void protectDate(int& a, int &b, int &c, int& d, int& e, int& f);
void limit_angle(float &a, float max);

/**
 * @brief 图像信息，用于线程之间的图像传输
 */

/**
 * @brief 线程管理类
 * 负责图像生成、图像处理、串口数据接收
 */
class ThreadControl
{
public:
    ThreadControl();          // 线程管理构造函数，用于线程中变量初始化
    void ImageProduce();      // 短焦摄像头获取图像线程
    void ImageProcess();      // 图像处理线程，用于自瞄，能量机关识别
    void GetSTM32();          // 用于接收电控发来的数据
    void ImageWrite();        // 用于图像保存线程
private:
    Mat image_;
    OtherParam other_param;
    MainWindow *w_;
    int last_mode = 0;

    bool end_thread_flag = false;
    bool debug_enable_flag = false;
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

