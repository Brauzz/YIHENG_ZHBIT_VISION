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
#include "../common/thread_control.h"
#include "../common/solve_angle/solve_angle.h"
#include "../base.h"
using namespace cv;
using namespace std;

/**
 * @brief 矩形类物体属性
 * 在逻辑识别部分需要修改原有旋转矩形属性
 * 在计算0-360角度上需要用到旋转矩形原始参数
 */
class object
{
public:
    RotatedRect rrect;
    RotatedRect origin_rrect;
};

/**
 * @brief BuffDetectTask 能量机关识别总任务，每帧调用
 * @param img 摄像头获取的RGB图像
 * @return 1是发现目标，0是未发现目标
 */

double calcDistanceFor2Point(Point2f p1, Point2f p2);

class BuffDetector
{
public:
    BuffDetector();
    BuffDetector(SolveAngle solve_angle);
    ~BuffDetector();
    bool DetectBuff(Mat& img);
    int8_t BuffDetectTask(Mat& img, OtherParam param);
    void getAngle(float &yaw, float &pitch){
        yaw = angle_x_;
        pitch = angle_y_;
    }

    /**
     * @brief 辨别能量机关旋转方向
     * 根据每次识别得到的角度进行滤波，判断能量机关旋转方向
     */
    void setFilter(size_t buff_size, float filter_angle_threshold){
        history_size_=buff_size;
        max_filter_value_=filter_angle_threshold;
    }
    int8_t getDirection(float angle);

private:
    int color_;
    int cap_mode_;
public:
    int buff_offset_x_ = 115;
    int buff_offset_y_ = 92;
    int world_offset_x_ = 500;
    int world_offset_y_ = 500;
    int color_th_ = 50;
    int gray_th_ = 50;
    float buff_angle_ = 0;

private:
    SolveAngle solve_angle_long_;
private:
    float angle_x_ = 0;
    float angle_y_ = 0;
    float distance_ = 0;
    vector<Point2f> points_2d;

private:
    vector<float> history_;
    size_t history_size_ = 10;
    float last_angle_ = 0;
    float max_filter_value_ = 15;

};


/**
 * @brief conversionAbsolutePoint
 * @param point_tmp
 * @param dst
 * @param offset
 * @param i1
 * @param i2
 * @param i3
 * @param i4
 */
void conversionAbsolutePoint(Point2f *point_tmp, vector<Point2f>& dst
                             ,Point2f offset
                             ,int8_t i1, int8_t i2, int8_t i3, int8_t i4);
