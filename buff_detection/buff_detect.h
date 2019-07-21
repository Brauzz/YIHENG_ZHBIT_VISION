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
#include "../mainwindow.h"
using namespace cv;
using namespace std;
#define BUFF_DETECT_DEBUG
#ifdef BUFF_DETECT_DEBUG
// ---- buff debug ----
#define DEBUG_DRAW_CONTOURS
#define DEBUG_PUT_TEST_TARGET
#define DEBUG_PUT_TEST_ANGLE
#define DEBUG_DRAW_TARGET
// ---- buff debug ----
#endif

typedef enum{UNKOWN,INACTION,ACTION}ObjectType;

/**
 * @brief 矩形类物体属性
 * 在逻辑识别部分需要修改原有旋转矩形属性
 * 在计算0-360角度上需要用到旋转矩形原始参数
 */
class Object
{
public:
    Object(){}

    RotatedRect small_rect_;
    RotatedRect big_rect_;
    vector<Point2f> points_2d_;
    float angle_;
    Point2f test_point_;
    int8_t direction_ = 1; // 1shun -1ni 0stop

    float length_scale_ = 3;
    float width_scale_ = -5.5;
    void DrawTarget(Mat &img)
    {
        if(type_ == INACTION)
            circle(img, small_rect_.center, 3, Scalar(0, 0, 255), -1);
        else if(type_ == ACTION)
            circle(img, small_rect_.center, 3, Scalar(255, 255, 255), -1);
        else
            circle(img, small_rect_.center, 3, Scalar(255, 255, 255), 1);
    }

    void UpdateOrder();
    void UpdataPredictPoint();
    int type_ = UNKOWN;
};

class AutoAttack
{
public:
    AutoAttack(){}
    int8_t run(bool find_target_flag, bool is_new_target, float angle_x, float angle_y);
private:
    int8_t mode_ = 1; // 1wait 2track 3shoot
    int lost_cnt_ = 0;
    int prepare_shoot_cnt = 0;
    double t1_;
};

double calcDistanceFor2Point(Point2f p1, Point2f p2);

/**
 * @brief BuffDetectTask 能量机关识别总任务，每帧调用
 * @param img 摄像头获取的RGB图像
 * @return 1是发现目标，0是未发现目标
 */
class BuffDetector
{
public:
    BuffDetector(){}
    BuffDetector(SolveAngle solve_angle);
    ~BuffDetector(){}
    void DebugPlotInit(MainWindow *w){
        w_ = w;
    }
    bool DetectBuff(Mat& img);
    int8_t BuffDetectTask(Mat& img, OtherParam param);
    void getAngle(float &yaw, float &pitch){
        yaw = angle_x_;
        pitch = angle_y_;
    }
    float getDistance(){
        return distance_;
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
    MainWindow *w_;
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

float Point_distance(Point2f p1,Point2f p2);
