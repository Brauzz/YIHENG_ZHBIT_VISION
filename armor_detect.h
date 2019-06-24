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
#include "common.h"

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
    void classification_lights(void);           // 分类装甲板左右灯条
    bool is_suitable_size(void) const;          // 判断可能的装甲板是否符合尺寸

    LED_Stick Led_stick[2];  // 装甲板的两个灯条
    float error_angle;       // 两个灯条的误差的角度
    Point2i center;          // 装甲板中心点
    Rect2i rect;             // 装甲板roi矩形
    int average_intensity;   // 装甲板roi的平均色彩强度
};

/**
 * @brief 装甲板类型确定
 * 通过检测历史数据判断装甲板类型，大装甲板和小装甲板
 */
class ArmorFilter {
public:
    ArmorFilter(int _filter_size = 5)
        : filter_size(_filter_size) {}

    void clear(){
        history.clear();
    }

    bool getResult(bool is_small){
        if (history.size() < filter_size){
            history.push_back(is_small);
        }
        else {
            history.push_back(is_small);
            history.pop_front();
        }

        int vote_cnt[2] = {0};
        for (std::list<bool>::const_iterator it = history.begin(); it != history.end(); ++it){
            *it == 0 ? ++vote_cnt[0] : ++vote_cnt[1];
        }

        if (vote_cnt[0] == vote_cnt[1])
            return is_small;
        return vote_cnt[0] > vote_cnt[1] ? 0 : 1;
    }

private:
    std::list<bool> history;
    int filter_size;
};

/**
 * @brief 装甲板检测任务函数，每帧调用
 * @param img 输入相机采集RGB图像
 * @param param 自瞄相关参数
 * @return 发现目标为1，未发现为0
 */
bool ArmorDetectTask(Mat &img, Parameter &param);

