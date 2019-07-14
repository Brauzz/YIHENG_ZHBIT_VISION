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
using namespace cv;

class ZeYuPredict
{
public:
    ZeYuPredict();
    ZeYuPredict(float Qp, float Qv, float Rp,float Rv,float dta, float pre_dta);
    void SetDelta(float dta);
    void SetPreDelta(float pre_dta);
    void ClearFilters(void);
    void InitFilters(float gim_angle);
    void setQRT(int Qp, int Qv, int Rp , int dta, float pre_dta);
    float run_position(float gim_angle);

    float getSpeed()
    {
        return v;
    }
    bool exit_flag;
private:
    Mat Qvar;
    Mat Rvar;
    Mat A;
    Mat B;
    Mat H;

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


