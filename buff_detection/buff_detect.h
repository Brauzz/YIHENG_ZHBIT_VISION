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
#include "iostream"
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
//#define TEST_OTSU
//#define AREA_LENGTH_ANGLE 2 // 1:area 2:length 3:diff_angle
#define FUSION_MINAREA_ELLIPASE
#define DIRECTION_FILTER
// ---- buff debug ----
#endif

#define DEFAULT 0
#define FIRE 3
#define RESET 4

typedef enum{UNKOWN,INACTION,ACTION}ObjectType;
typedef enum{restore_center=6,follow=-1,shoot=3}mode_buff;

/**
 * @brief 矩形类物体属性
 * 在逻辑识别部分需要修改原有旋转矩形属性
 * 在计算0-360角度上需要用到旋转矩形原始参数
 */
class Object
{
public:
    Object(){}

    RotatedRect fitEllipse_rect;
    RotatedRect minArea_rect;
    RotatedRect small_rect_;
    RotatedRect big_rect_;
    vector<Point2f> points_2d_;
    float angle_;
    Point2f test_point_;
    float diff_angle;
    int direction_ = 1; // 1shun -1ni 0stop

    float length_scale_ = 3;
    float width_scale_ = -3;

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
    void KnowYourself(Mat &img);
    void Indeed_smallrect();
    void UpdataPredictPoint();

    int type_ = UNKOWN;
};

class AutoAttack
{
public:
    AutoAttack(){}
    int run(bool find_target_flag, float angle_x, float angle_y, int target_size, float gimbal, int move_static);
    int adjust_control(bool find_target_flag, int move_static,int target_size);
private:
    int control_=restore_center;
    int buff_mode;
    int t_tocul=0; //0=little; 1=big
    int restore_count=0;
    int center_buff=0;
    int count_center=0;
};


class FireTask{
public:
    FireTask(){}
    // 开火条件是当输出的角度趋近于０一段时间开火
    int run(Point2f current_angle, bool is_change_target)
    {
        // 获得发射机会的条件
        if(wait_action_flag == true ){// 重复激活
            if(!is_change_target){
                double t2 = getTickCount();
                double t = (t2 - change_time)*1000 / getTickFrequency();
                if(t > 1000){
                    printf("重复激活\r\n");
                    change_time = getTickCount();
#ifndef NO_REPEAT_FIRE
                    shoot_chance_ = true;
#endif
                }
            }else{
                wait_action_flag = false;
            }
        }

        if(is_change_target && wait_action_flag== false){
            shoot_chance_ = true;
            change_time = getTickCount();
            wait_action_flag = true;
        }
        // 控制发射条件
        int command = DEFAULT;
        if(fabs(current_angle.x) < limit_angle_x_
                && fabs(current_angle.y) < limit_anlge_y_)
        {
            // 满足小于一段时间计数
            cnt_ ++;
        }else {
            // 不满足条件加速减时间
            cnt_ -= 3;
            if(cnt_<0) cnt_ = 0;
        }
        if(cnt_ > max_cnt_)
        {
            cnt_ = 0;
            if(shoot_chance_ == true)
            {
#ifndef NO_FIRE
                command = FIRE;
#else
                command = DEFAULT;
#endif
                fire_cnt++;
//                printf("开火:%d\r\n", fire_cnt);
                shoot_chance_ = 0;
            }else {
                command = DEFAULT;
            }
        }else{
            command = DEFAULT;
        }
        return command;
    }

    void set_fire_chance(){
        shoot_chance_ = true;
        change_time = getTickCount();
        wait_action_flag = true;
    }
private:
    // 获得发射机会的参数
    Point2f last_angle_;
    bool wait_action_flag = true;
    double change_time;
    bool shoot_chance_ = true;


    // 控制开火的参数
    int cnt_ = 0;
    int fire_cnt = 0;
    int max_cnt_ = 50;             // 满足条件次数
    float limit_angle_x_ = 2.0f;    // 条件角度阈值
    float limit_anlge_y_ = 2.0f;
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
    int max_cnt_ = 30;   // 最大丢失目标次数
};


class ProPrediceTask{
public:
    ProPrediceTask(){}

    int run(int fire_cnt, int action_cnt){
        //退出超预测模式的条件
        if(pro_predict_mode_flag == true)
        {
            // 超时时间退出
            double t2 = getTickCount();
            double t = (t2 - t1)*1000/getTickFrequency();
            if(t > 1000)
                pro_predict_mode_flag = false;

            // 开火后退出
            if(pro_fire_cnt != fire_cnt)
            {
                pro_predict_mode_flag = false;
                pro_fire_cnt = fire_cnt;
            }
        }

        // 进入超预测模式条件
        if(pro_fire_cnt != fire_cnt)
        {
            if(action_cnt == 3)
            {
                // 激活数量变成３进入超预测模式
                t1 = getTickCount();
                pro_predict_mode_flag = true;
            }
            pro_fire_cnt = fire_cnt;
        }
        return pro_predict_mode_flag;
    }

private:
    int pro_fire_cnt = 0;   // 用于判断开火变化的计数
    bool pro_predict_mode_flag = false;
    double t1;

};


class AutoControl
{
public:
    AutoControl(){}
    int run(float current_yaw, float &current_pit,int find_flag, float diff_buff_angle){
        int command_tmp = DEFAULT;
        // 开火任务启动
        bool is_change_target = false;
        if(fabs(diff_buff_angle) > 40  && fabs(diff_buff_angle) < 350){
            filter_flag = true;
        }else{
            if(filter_flag == true){
                printf("change\r\n");
                is_change_target = true;
                filter_flag = false;
            }
        }

        command_tmp = fire_task.run(Point2f(current_yaw, current_pit), is_change_target);
        if(command_tmp != DEFAULT){
            set_fire(command_);
            fire_cnt++;
            //            printf("fire:%d\r\n", fire_cnt);
            //            INFO(command_);
            debug = false;
            return command_;
        }
        // 复位任务启动
        command_tmp = reset_task.run(find_flag);
        if(command_tmp != DEFAULT){
            // 复位绝对角度
            set_reset(command_);
            if(debug == false){
                printf("reset!!!\r\n");
            }
            debug = true;
            current_pit = -5;
            fire_task.set_fire_chance();// 复位获得一次开火机会
            return command_;
        }
        // 通过上面开火任务及复位任务还是得到默认指令，则判断跟随还是不跟随
        if(command_tmp == DEFAULT)
        {
            if(find_flag == 1)
            {
                set_follow(command_);
            }else
            {
                set_no_follow(command_);
            }
        }

        //        pro_predict_flag_ = pro_predict_task.run(fire_cnt, action_cnt);
        return command_;
    }

    bool GetProPredictFlag(){
        return pro_predict_flag_;
    }
private:
    // 设置命令相关函数
    // 置1用 |    清零用&   跟随位0x01 开火位0x02 复位位0x04
    void set_follow(int &command){
        command &= ~0x04;
        command |= 0x01;

    }
    void set_no_follow(int &command){
        command &= ~0x04;
        command &= ~0x01;

    }
    void set_fire(int &command){
        command |= 0x01;
        command ^= 0x02;
    }

    void set_reset(int &command){
        command |= 0x04;
    }

public:
    FireTask fire_task;
    ResetTask reset_task;
    ProPrediceTask pro_predict_task;


    float last_diff_buff_angle_;
    int diff_angle_cnt_ = 0;
    bool filter_flag = false;

    bool pro_predict_flag_ = false;
    int command_ = 0;
    int fire_cnt = 0;

    bool debug = false;
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
    int getSimpleDirection(float angle);


private:
    bool DetectBuff(Mat& img, OtherParam other_param);
    void setFilter(size_t buff_size, float filter_angle_threshold){
        history_size_=buff_size;
        max_filter_value_=filter_angle_threshold;
    }
    void trigger(int target_size,int move_static);
    int GetIndex(Mat &img, Object object, vector<Rect> all_rect);

    // 外部参数
private:
    int color_;
    float gimbal;

    // debug参数
public:
    int buff_offset_x_ =130;//id:3 69;// id:2 130;
    int buff_offset_y_ =83;//id:3 100;// id:2 135;
    int world_offset_x_ = 750;
    int world_offset_y_ = 450;
    int color_th_ = 15;
    int gray_th_ = 50;
    float buff_angle_ = 0;
    float diff_angle_ = 0;
    int area_ratio_ = 500;
    //相关类申明
private:
    SolveAngle solve_angle_long_;
    AutoAttack auto_attack;
    AutoControl auto_control;
    AutoAttack attack;
    MainWindow *w_;

private:
    float angle_x_ = 0;
    float angle_y_ = 0;
    float last_angle_x_ = 0;
    float last_angle_y_ = 0;
    float distance_ = 0;
    vector<Point2f> points_2d;
    int action_cnt_ = 0;
    vector<float> vec_diff_angle;

private:
    float d_angle_ = 0;
    float r = 0.1;
    int find_cnt = 0;
    vector<float> history_;
    size_t history_size_ = 10;
    float last_angle_ = 0;
    float max_filter_value_ = 15;
    int direction_tmp=0;

    // 超预测相关参数
private:
    int or_fire_cnt_ = 0;
    bool pro_predict_mode_flag = false;



    int command = 0;
};

double Point_distance(Point2f p1,Point2f p2);



