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
#include "boost/timer.hpp"

#include "camera_device.h"
#include "armor_detect.h"
#include "buff_detect.h"
#include "serial_port.h"
#include "solve_angle.h"
#include "predict.h"
#include "common.h"

using namespace cv;
using namespace std;

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
public:

    bool short_camera_flag;
    bool long_camera_flag;

//    int8_t mode_;           // 视觉模式，0是自瞄模式，1是能量机关模式
    //    bool cap_mode_;       // 摄像头类型，0是短焦摄像头，1是长焦摄像头
//    int8_t color_;          // 我方车辆颜色，0是蓝色，1是红色。用于图像预处理
    float bullet_speed_;    // 子弹发射速度(m/s)
    int8_t cancel_kalman_;  // 取消预测标志位，当预测不稳定时，操作手可以取消预测
    // kalman滤波的参数
    int km_Qp = 500, km_Qv = 1, km_Rp = 1, km_Rv = 1;
    int km_t = 1, km_pt = 70;  // (ms)
private:

};

/**
 * @brief 图像信息，用于线程之间的图像传输
 */
struct ImageData
{
    Mat img;
    float gimbal_data;
    //    unsigned int frame;
};

struct OtherParam
{
    int8_t color;
    int8_t mode;
    int8_t cap_mode;
};

void protectDate(int& a, int &b, int &c, int& d, int& e, int& f);
void limit_angle(float &a, float max);

class GimbalDataProcess
{
public:
    GimbalDataProcess():
        last_gimbal_yaw(0.0f),
        count (0){}

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
