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
#include "predict.h"
ZeYuPredict::ZeYuPredict(){}

ZeYuPredict::ZeYuPredict(float Qp, float Qv, float Rp,float Rv,float dta, float pre_dta)
{
    Qvar = (Mat_<float>(2,2)<< Qp, 0, 0, Qv);
    delta = dta;
    pre_delta = pre_dta;
    A = (Mat_<float>(2,2) << 1, dta, 0, 1);
    B = (Mat_<float>(2,2) << 1, 1/2*pow(dta,2), 0, 1);
    H = (Mat_<float>(1,2) << 1, 0);
    Rvar = static_cast<double>(Rp);
    //state
    po = (Mat_<float>(2,2)<< 1000, 1000, 1000, 1000);//0.00623 don't worried, because it can update
    po_pre = po;
    x_pre = (Mat_<float>(2,1)<< 0,0);
    kg = (Mat_<float>(2,1)<< 0,0);
    xkf = (Mat_<float>(2,1)<< 0,0);
    predict = 0;
    v = 0;
    last_v = 0;
}


float ZeYuPredict::run_position(float gim_angle)
{
    Mat I = Mat::eye(2,2,CV_32F);
    x_pre = A * xkf ;
    po_pre = A * po * A.t() + Qvar;
    kg = po_pre * H.t() * (H * po_pre * H.t() + Rvar).inv();
    xkf = x_pre + kg* (static_cast<double>(gim_angle) - H * x_pre);
    v = xkf.at<float>(1,0);
//    float error = v - last_v;
//    last_v = v;
//    if(abs(error)>0.5f)
//    {
//        v = 0;
//    }
//    std::cout << v << std::endl;
    predict = xkf.at<float>(0,0) + v * pre_delta;
    po = (I - kg * H) * po_pre;
    return predict;
}


void ZeYuPredict::setQRT(int Qp, int Qv, int Rp ,int dta, float pre_dta)
{
    Qvar = (Mat_<float>(2,2)<< Qp*0.001, 0, 0, Qv*0.001);
    Rvar = Rp*0.001;
    delta = static_cast<float>(dta);
    pre_delta = pre_dta;
}

void ZeYuPredict::SetDelta(float dta)
{
    delta = dta;
}

void ZeYuPredict::SetPreDelta(float pre_dta)
{
    pre_delta = pre_dta;
}

void ZeYuPredict::InitFilters(float gim_angle)
{
    x_pre = static_cast<double>(gim_angle);
}

void ZeYuPredict::ClearFilters()
{
    po = (Mat_<float>(2,2)<< 1000, 0, 0, 1000);// don't worried, because it can update
    po_pre = po;
    x_pre = (Mat_<float>(2,1)<< 0,0);
    kg = (Mat_<float>(2,1)<< 0,0);
    xkf = (Mat_<float>(2,1)<< 0,0);
    predict = 0;
}

