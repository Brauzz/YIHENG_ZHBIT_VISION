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

#define DEFAULT -1
#define FIRE 3
#define RESET 4


typedef enum{UNKOWN,INACTION,ACTION}ObjectType;
typedef enum{restore_center,follow,shoot}mode_buff;

static int target_size=0;
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
    int direction_ = 1; // 1shun -1ni 0stop

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

    void UpdateOrder(Point2f offset_point);
    void UpdataPredictPoint();
    int type_ = UNKOWN;
};

class AutoAttack
{
public:
    AutoAttack(){}
    int run(bool find_target_flag, float angle_x, float angle_y, int target_size, float gimbal, int move_static);
private:
    int control_=restore_center;
    int buff_mode;
    int t_tocul=0; //0=little; 1=big
    int restore_count=0;
};


class FireTask{
public:
    FireTask(){}
    // 开火条件是当输出的角度趋近于０一段时间开火
    int run(float current_yaw, float current_pit)
    {
        int command = DEFAULT;
        if(current_yaw < limit_angle_x_
                && current_pit < limit_anlge_y_)
        {
            // 满足小于一段时间计数
            cnt ++;
        }else {
            // 不满足条件加速减时间
            cnt -= 3;
            if(cnt<0) cnt = 0;
            // 获得一次开火的机会
            shoot_chance = true;
        }
        // 控制发射条件
        if(cnt > max_cnt_)
        {
            cnt = 0;
            if(shoot_chance == true)
            {
                command = FIRE;
                shoot_chance = 0;
            }else {
                command = DEFAULT;
            }
        }else{
            command = DEFAULT;
        }
        return command;
    }
private:
    int cnt = 0;
    bool shoot_chance = true;

    int max_cnt_ = 100;             // 满足条件次数
    float limit_angle_x_ = 0.1f;    // 条件角度阈值
    float limit_anlge_y_ = 0.1f;
};

class ResetTask{
public:
    ResetTask(){}
    // 复位归中条件是，丢失目标超过几帧
    int run(bool find_flag)
    {
        int command = DEFAULT;
        if(find_flag == false){
            cnt++;
        }else{
            cnt = 0;
        }
        // 判断归中条件
        if(cnt > max_cnt_)
        {
            command = RESET;
        }else{
            command = DEFAULT;
        }
        return command;
    }
private:
    int cnt = 0;
    int max_cnt_ = 3;   // 最大丢失目标次数
};


class AutoControl
{
public:
    AutoControl(){}
    int run(float current_yaw, float &current_pit,int find_flag){
        int command = DEFAULT;
        command = fire_task.run(current_yaw, current_pit);
        if(command != DEFAULT)
            return command;

        command = reset_task.run(find_flag);
        if(command != DEFAULT){
            // 复位绝对角度
            current_pit = 10;
            return command;
        }


        return find_flag;
    }

public:
    FireTask fire_task;
    ResetTask reset_task;

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
    BuffDetector(){
        solve_angle_long_ = SolveAngle(CAMERA1_FILEPATH, LONG_X, LONG_Y, LONG_Z, PTZ_TO_BARREL);
    }
    ~BuffDetector(){}
    void DebugPlotInit(MainWindow *w){
        w_ = w;
    }

    int BuffDetectTask(Mat& img, OtherParam param);
    void getAngle(float &yaw, float &pitch){
        yaw = angle_x_;
        pitch = angle_y_;
    }

    /**
     * @brief 辨别能量机关旋转方向
     * 根据每次识别得到的角度进行滤波，判断能量机关旋转方向
     */
    float getDistance(){
        return distance_;
    }


    int getDirection(float angle);

private:
    bool DetectBuff(Mat& img, OtherParam other_param);
    void setFilter(size_t buff_size, float filter_angle_threshold){
        history_size_=buff_size;
        max_filter_value_=filter_angle_threshold;
    }


private:
    int color_;
    float gimbal;
public:
    int buff_offset_x_ = 51;
    int buff_offset_y_ = 0;
    int world_offset_x_ = 500;
    int world_offset_y_ = 500;
    int color_th_ = 50;
    int gray_th_ = 50;
    float buff_angle_ = 0;
    AutoAttack attack;
    int target_size;
    int move_static;
    int do_you_find_inaction=0;
private:
    SolveAngle solve_angle_long_;
    AutoAttack auto_attack;
    AutoControl auto_control;
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



