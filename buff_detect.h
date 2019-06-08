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
 * @brief 辨别能量机关旋转方向
 * 根据每次识别得到的角度进行滤波，判断能量机关旋转方向
 */
class DirectionFilter
{
public:
    DirectionFilter(size_t buff_size = 10, float filter_angle_threshold = 15)

    {
        history_size=buff_size;
        max_filter_value=filter_angle_threshold;
    }
    bool getDirection(float angle)
    {
        float error_angle = last_angle - angle;
//        cout << "error_angle" << error_angle << endl;
        last_angle = angle;
        if(fabs(error_angle) < max_filter_value && fabs(error_angle) < 1e-6f)
        {
            if(history.size() < history_size)
            {
                history.push_back(error_angle);
            }else {
                history.push_back(error_angle);
                history.erase(history.begin());
            }
        }
        std::vector<float>::iterator iter;
        float sum = 0.0;
        for (iter=history.begin();iter!=history.end();iter++){
            sum += *iter;
        }
        sum /= history.size();
//        cout << "sum " << sum << endl;

        if(sum >= 0)
            return 0;   // shun
        else
            return 1;   // ni

    }
private:
    vector<float> history;
    size_t history_size;
    float last_angle;
    float max_filter_value;
};

/**
 * @brief BuffDetectTask 能量机关识别总任务，每帧调用
 * @param img 摄像头获取的RGB图像
 * @param target_2d_point 图像上目标的4个顶点
 * @param our_color 自身车辆颜色
 * @param offset_tvec 图像左边点平移补偿量
 * @param theta_angle 得到的目标角度
 * @return 1是发现目标，0是未发现目标
 */
bool BuffDetectTask(Mat &img, vector<Point2f> &target_2d_point, int8_t our_color, Point2f offset_tvec, float &theta_angle);

double calcDistanceFor2Point(Point2f p1, Point2f p2);

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
