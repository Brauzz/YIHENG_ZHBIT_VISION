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
#include "../mainwindow.h"
using namespace std;
using namespace cv;

#ifdef DEBUG_ARMOR_DETECT
// ------ debug armor ------
//#define SHOW_BINARY_IMAGE
//#define SHOW_LIGHT_PUT_TEXT
#define SHOW_ARMOR_PUT_TEXT
#define SHOW_LIGHT_CONTOURS
#define SHOW_ROI_RECTANGLE
#define SHOW_DRAW_SPOT
#define SHOW_DRAW_RECT
// ------ debug armor ------
#endif

#define FAST_DISTANCE
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

    void draw_rect( Mat& img, Point2f roi_offset_poin) const;    // 画出装甲板
    void draw_spot(Mat &img, Point2f roi_offset_point) const;
    int get_average_intensity(const Mat& img) ; // 计算装甲板roi平均色彩强度，用于筛选装甲板中心有灯条
    void max_match(vector<LED_Stick>& LED, size_t i, size_t j); // 灯条匹配算法
    bool is_suitable_size(void) const;          // 判断可能的装甲板是否符合尺寸

    LED_Stick Led_stick[2];  // 装甲板的两个灯条
    float error_angle;       // 两个灯条的误差的角度
    Point2i center;          // 装甲板中心点
    Rect2i rect;             // 装甲板roi矩形
    int average_intensity;   // 装甲板roi的平均色彩强度
};


/**
 * @brief 装甲板检测器
 */
class ArmorDetector
{
public:
    ArmorDetector(){
        solve_angle_ = SolveAngle(CAMERA0_FILEPATH, SHOR_X, SHOR_Y, SHOR_Z, PTZ_TO_BARREL); // 短焦角度解算
        solve_angle_long_ = SolveAngle(CAMERA1_FILEPATH, LONG_X, LONG_Y, LONG_Z, PTZ_TO_BARREL); // 长焦角度解算
        predict_ = Predictor(30); // 二次多项式拟合（国赛停用）
        zeyu_predict_ = ZeYuPredict(0.01f, 0.01f, 0.01f, 0.01f, 1.0f, 3.0f);    // 二阶ｋａｌｍａｎ预测，视觉（国赛停用）
        t_start_ = getTickCount();
    }
    ~ArmorDetector(){}

    /**
     * @brief 上位机初始化
     */
    void DebugPlotInit(MainWindow *w){
        w_ = w;
    }

    /**
     * @brief chooseCamera 选择相机类型，长短焦切换逻辑
     * @param 短距离阈值(mm)
     * @param 长距离阈值(mm)
     * @param 上一次摄像头模式
     * @return 返回本次摄像头类型 1长焦 0短焦
     */
    bool chooseCamera(int short_distance, int long_distance, bool last_mode);


    /**
     * @brief ArmorDetectTask 自瞄任务函数（识别，角度解算）
     * @param img
     * @param other_param 其他参数，其中使用到color颜色和cap_mode摄像头类型
     * @return 返回命令 0没发现 1发现
     */
    int ArmorDetectTask(Mat &img, OtherParam other_param);
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

private:
    /**
     * @brief GetRoi 获取图像ROI区域
     * @param img
     * @return 返回感兴趣区域的矩形
     */
    Rect GetRoi(const Mat &img);
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

    /**
     * @brief DetectArmor 装甲板识别函数
     * @param img
     * @param roi_rect
     * @return
     */
    bool DetectArmor(Mat &img, Rect roi_rect);
    /**
     * @brief 装甲板类型确定
     * 通过检测历史数据判断装甲板类型，大装甲板和小装甲板
     */
    bool getTypeResult(bool is_small);
    void setFilter(int filter_size){
        filter_size_ = filter_size;
    }
    void clear(){
        history_.clear();
    }

private:
    // 外部参数
    int color_;
    int cap_mode_;

public:
    // ｋａｌｍａｎ滤波预测参数
    int km_Qp_ = 1000;
    int km_Qv_ = 1;
    int km_Rp_ = 1;
    int km_Rv_ = 1;
    int km_t_ = 1;
    int km_pt_ = 60;
    float last_angle = 0;
    float last_v = 0;
    float last_last_v = 0;

public:
    // 调试参数
    int short_offset_x_ = 100;
    int short_offset_y_ = 100;
    int long_offset_x_ = 100;
    int long_offset_y_ = 100;
    int color_th_ = 16;
    int gray_th_ = 60;


private:
    // 相关类声明
    SolveAngle solve_angle_;
    SolveAngle solve_angle_long_;
    ZeYuPredict zeyu_predict_;
    Predictor predict_;
    double t_start_;
    MainWindow *w_;
    Kalman1 kalman;

private:
    // ｒｏｉ参数
    Rect last_target_;
    int lost_cnt_ = 0;
    int detect_cnt_ = 0;

private:
    float dist_ = 3000; // 通过距离更换相机
    float r_ = 0.5; // 距离刷新率 (0-1)
    int update_cap_cnt = 0; // 用于强制限制相机更新频率
    float distance_ = 0;
    float angle_x_ = 0;
    float angle_y_ = 0;
    vector<Point2f> points_2d_;

private:
    // 判断大小装甲板类型相关参数
    std::list<bool> history_;
    int filter_size_ = 5;
    bool is_small_;

};




