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
#include <list>
using namespace cv;

class ZeYuPredict
{
public:
    ZeYuPredict();
    ZeYuPredict(float Qp, float Qv, float Rp,float Rv,float dta, float pre_dta);
    void ClearFilters(void);
    void setQRT(int Qp, int Qv, int Rp , int dta, float pre_dta);
    void setdelta(float t){
        delta = t;
    }
    float run_position(float gim_angle);
    float run_position(float gimbal_anlge, float v);
    float run_position(float gimbal_anlge, float v, float u);
    float getSpeed()
    {
        return v;
    }
    bool exit_flag;
private:
    Mat Qvar;
    Mat R;      // 单观测变量方差
    Mat Rvar;   // 两个观测变量协方差
    Mat A;
    Mat B;
    Mat H;      // 单变量观测矩阵
    Mat H2;     // 两变量观测矩阵
    float delta;
    float pre_delta;

    //state
    Mat po;         //error covarience
    Mat po_pre;     //error co
    Mat x_pre;    //the last angle
    Mat kg;       //kalman gain
    Mat xkf;      //kalman angle
    float v;
    float last_v;
    float predict;  //predict angle
};


class Predictor {
public:
    Predictor(int _history_size = 5)
        : history_size(_history_size){}

    bool setRecord(double value, double time);

    // Using history data to predict the query value
    // (fit the data using quadratic curve at min MSE criterion)
    double predict(double time);
    void clear(){
        history_value.clear();
        history_time.clear();
    }

private:
    std::list<double> history_value;
    std::list<double> history_time;
    int history_size;
};
