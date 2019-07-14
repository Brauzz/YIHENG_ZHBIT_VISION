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
#include "../common/thread_control.h"
#include "../common/solve_angle/solve_angle.h"
#include "../common/filter/predict.h"
#include "../base.h"
using namespace std;
using namespace cv;
/**
 * @brief 装甲板灯条相关数据信息
 */

class LED_Stick{
public:

    LED_Stick():matched(false){}

    LED_Stick(const RotatedRect& R){
        rect.angle = R.angle;
        rect.center = R.center;
        rect.size = R.size;
        matched = false;
    }

    RotatedRect rect;   // 装甲板灯条相关数据信息
    bool matched;       // 匹配状态， 用于灯条匹配
    size_t match_index; // 匹配对应的灯条序号， 用于灯条匹配
    float match_factor; // 匹配强度， 用于灯条匹配
};

/**
 * @brief 装甲板相关数据信息
 */
class armor{
public:
    armor();
    armor(const LED_Stick& L1, const LED_Stick& L2);

    void draw_rect( Mat& img) const;    // 画出装甲板
    void draw_spot(Mat &img) const;
    int get_average_intensity(const Mat& img) ; // 计算装甲板roi平均色彩强度，用于筛选装甲板中心有灯条
    void max_match(vector<LED_Stick>& LED, size_t i, size_t j); // 灯条匹配算法
    bool is_suitable_size(void) const;          // 判断可能的装甲板是否符合尺寸

    LED_Stick Led_stick[2];  // 装甲板的两个灯条
    float error_angle;       // 两个灯条的误差的角度
    Point2i center;          // 装甲板中心点
    Rect2i rect;             // 装甲板roi矩形
    int average_intensity;   // 装甲板roi的平均色彩强度
};



class ArmorDetector
{
public:
    ArmorDetector(){}
    ArmorDetector(SolveAngle solve_short, SolveAngle solve_long, ZeYuPredict zeyu_predict);
    ~ArmorDetector(){}

    bool chooseCamera(int short_distance, int long_distance, bool last_mode)
    {
        dist_ = (1-r_)*dist_ + r_*distance_ + 1;
        if(dist_ > long_distance && last_mode == 0)
            return 1;
        else if(dist_ < short_distance && last_mode == 1)
            return 0;
        else
            return last_mode;
    }

    /**
     * @brief 装甲板检测任务函数，每帧调用
     * @param img 输入相机采集RGB图像
     * @param param 自瞄相关参数
     * @return 发现目标为1，未发现为0
     */
    Mat setImage(const cv::Mat &src);
    bool makeRectSafe(cv::Rect & rect, cv::Size size){
        if (rect.x < 0)
            rect.x = 0;
        if (rect.x + rect.width > size.width)
            rect.width = size.width - rect.x;
        if (rect.y < 0)
            rect.y = 0;
        if (rect.y + rect.height > size.height)
            rect.height = size.height - rect.y;
        if (rect.width <= 0 || rect.height <= 0)
            return false;
        return true;
    }
    bool DetectArmor(Mat &img);
    int8_t ArmorDetectTask(Mat &img, OtherParam other_param);
    void DrawTarget(Mat &img)
    {
        if(!points_2d_.empty())
        {
            line(img, points_2d_[0],points_2d_[2],Scalar(255,100,0), 3);
            line(img, points_2d_[1],points_2d_[3],Scalar(255,100,0), 3);
        }
    }
    void getAngle(float &yaw, float &pitch)
    {
        yaw = angle_x_;
        pitch = angle_y_;
    }

public:
    void setFilter(int filter_size){
        filter_size_ = filter_size;
    }

    void clear(){
        history_.clear();
    }

    /**
     * @brief 装甲板类型确定
     * 通过检测历史数据判断装甲板类型，大装甲板和小装甲板
     */
    bool getTypeResult(bool is_small);
private:
    int color_;
    int cap_mode_;
public:
    int km_Qp_ = 1000;
    int km_Qv_ = 1;
    int km_Rp_ = 1;
    int km_Rv_ = 1;
    int km_t_ = 1;
    int km_pt_ = 60;
public:
    int short_offset_x_ = 120;
    int short_offset_y_ = 100;
    int long_offset_x_ = 85;
    int long_offset_y_ = 100;
    int color_th_ = 100;
    int gray_th_ = 100;

private:
    SolveAngle solve_angle_;
    SolveAngle solve_angle_long_;
    ZeYuPredict zeyu_predict_;
private:
    RotatedRect last_target;
    Rect detect_rect_;
    int lost_cnt = 0;
private:
    float dist_ = 3000;
    float r_ = 0.5;
    float distance_ = 0;
    float angle_x_ = 0;
    float angle_y_ = 0;
    bool is_small_;
    vector<Point2f> points_2d_;

private:
    std::list<bool> history_;
    int filter_size_ = 5;

};




