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
    B = (Mat_<float>(2,1) << 1/2*pow(dta,2), dta);
    H = (Mat_<float>(1,2) << 1, 0);
    H2 = (Mat_<float>(1, 2) << 1, 1);
    R = static_cast<double>(Rp);
    Rvar = (Mat_<float>(2,2)<< Rp, 0, 0, Rv);

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
    kg = po_pre * H.t() * (H * po_pre * H.t() + R).inv();
    xkf = x_pre + kg* (static_cast<double>(gim_angle) - H * x_pre);
    v = xkf.at<float>(1,0);
    predict = xkf.at<float>(0,0);
    po = (I - kg * H) * po_pre;
    return predict;
}

float ZeYuPredict::run_position(float gimbal_anlge, float v)
{
    Mat Z = (Mat_<float>(2,1)<< gimbal_anlge, v);
    Mat I = Mat::eye(2,2,CV_32F);
    x_pre = A * xkf ;
    po_pre = A * po * A.t() + Qvar;
    kg = po_pre * H2.t() * (H2 * po_pre * H2.t() + Rvar).inv();
    xkf = x_pre + kg* (Z - H2 * x_pre);
    v = xkf.at<float>(1,0);
    predict = xkf.at<float>(0,0) + v * pre_delta;
    po = (I - kg * H2) * po_pre;
    return predict;
}

float ZeYuPredict::run_position(float gimbal_anlge, float v, float u)
{
    Mat Z = (Mat_<float>(2,1)<< gimbal_anlge, v);
    Mat I = Mat::eye(2,2,CV_32F);
    x_pre = A * xkf + B*u;
    po_pre = A * po * A.t() + Qvar;
    kg = po_pre * H2.t() * (H2 * po_pre * H2.t() + Rvar).inv();
    xkf = x_pre + kg* (Z - H2 * x_pre);
    v = xkf.at<float>(1,0);
    predict = xkf.at<float>(0,0) + v * pre_delta;
    po = (I - kg * H2) * po_pre;
    return predict;
}



void ZeYuPredict::setQRT(int Qp, int Qv, int Rp ,int dta, float pre_dta)
{
    Qvar = (Mat_<float>(2,2)<< Qp*0.001, 0, 0, Qv*0.001);
    Rvar = Rp*0.001;
    delta = static_cast<float>(dta);
    pre_delta = pre_dta;
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



// https://blog.csdn.net/weixin_44344462/article/details/88850409
bool Predictor::setRecord(double value, double time){
    if (history_value.size() < history_size){
        history_time.push_back(time);
        history_value.push_back(value);
    }
    else {
        history_time.push_back(time);
        history_value.push_back(value);
        history_time.pop_front();
        history_value.pop_front();
    }
}

double Predictor::predict(double time){
    std::list<double>::const_iterator it_in = history_time.begin();
    double latest_value = history_value.back();
//    if(abs(latest_value) < 5.0 || history_value.size() < history_size)
//        return latest_value;
//    if(history_time.back() - *it_in > 150.0)
//        return latest_value;
    std::list<double>::const_iterator it_out = history_value.begin();
    std::list<double>::const_iterator prev_out = it_out;
    double max_o = -500000, min_o = 500000;
//    for(std::list<double>::const_iterator it = it_out, it_i = it_in; it != history_value.end(); ++it, ++it_i){
//        if(max_o < *it)
//            max_o = *it;
//        if(min_o > *it)
//            min_o = *it;
//        if(abs(*it - *prev_out) > 5.0){
//            return latest_value;
//        }
//        prev_out = it;
//        //printf("(%2f,%2f) ", *it, *it_i);
//    }
   // printf("\n");
//    if (max_o - min_o < 3.0)      // angle gap must lager than 3 degree
//        return latest_value;

    Mat A(history_size,3,CV_64F);
    Mat b(history_size,1,CV_64F);
    double * b_data = (double *) b.data;
    double * A_data = (double *) A.data;
    // Ａ矩阵 Ｂ矩阵赋值
    for (; it_in != history_time.end(); ++A_data, ++b_data, ++it_in, ++it_out){
        *A_data = (*it_in-time) * (*it_in-time);
        *(++A_data) = (*it_in-time);
        *(++A_data) = 1;
        *b_data = *it_out;
    }
    // 方程 a∗X2+b∗X+c=y
    // 矩阵形式  A*w = b  =>  w = (A_t * A).inverse * b
    Mat A_t = A.t();
    Mat w = (A_t*A).inv()*A_t * b;
    Mat q = (Mat_<double>(1,3) << 0, 0, 1);
    Mat ret = q*w;

    double predict_angel = ret.at<double>(0);
    const double max_gap = 10.0;
    if(predict_angel - latest_value > max_gap)
        predict_angel = latest_value + max_gap;
    else if(predict_angel - latest_value < -max_gap)
        predict_angel = latest_value - max_gap;
    return predict_angel;

}

