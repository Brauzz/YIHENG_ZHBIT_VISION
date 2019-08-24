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
#define AREA_LENGTH_ANGLE 1 // 1:area 2:length 3:diff_angle
#define FUSION_MINAREA_ELLIPASE
#define DIRECTION_FILTER
#define IMSHOW_2_ROI
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
    void DrawTarget(Mat &img)
    {
        if(type_ == INACTION)
            circle(img, small_rect_.center, 3, Scalar(0, 0, 255), -1);
        else if(type_ == ACTION)
            circle(img, small_rect_.center, 3, Scalar(255, 255, 255), -1);
        else
            circle(img, small_rect_.center, 3, Scalar(255, 255, 255), 1);
    }

    void UpdateOrder(); // 更新能量机关装甲板的绝对位置
    void KnowYourself(Mat &img);    //判断能量机关扇叶的状态（激活　未激活）

    RotatedRect small_rect_;    // 能量机关扇叶内轮廓
    RotatedRect big_rect_;  // 能量机关扇叶外轮廓
    vector<Point2f> points_2d_; // ｐｎｐ角度解算的四个点
    float angle_;
    float diff_angle;
    int type_ = UNKOWN;
};

/**
 * @brief 能量机关自动开火任务
 */
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
                if(t > repeat_time){
//                    printf("重复激活\r\n");
                    change_time = getTickCount();
#ifndef NO_REPEAT_FIRE
                    if(repeat_fire_flag == 1){
                        shoot_chance_ = true;
                        printf("repeat\r\n");
                    }else{
                        shoot_chance_ = false;
                         printf("repeat-default\r\n");
                    }
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
                if(fire_flag == 1){
                    command = FIRE;
                    printf("fire\r\n");
                }else{
                     command = DEFAULT;
                     printf("fire-default\r\n");
                }

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
public:
    // 获得发射机会的参数
    Point2f last_angle_;
    bool wait_action_flag = true;
    double change_time;
    bool shoot_chance_ = true;
    int repeat_time = REPEAT_FIRE_TIME;
    // debug
    int fire_flag = 0;
    int repeat_fire_flag = 0;

    // 控制开火的参数
    int cnt_ = 0;
    int fire_cnt = 0;
    int max_cnt_ = 50;             // 满足条件次数
    float limit_angle_x_ = 2.0f;    // 条件角度阈值
    float limit_anlge_y_ = 2.0f;
};

/**
 * @brief 能量机关复位任务
 */
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
public:
    int cnt = 0;
    int max_cnt_ = 30;   // 最大丢失目标次数
};

/**
 * @brief 能量机关自动控制
 */
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
            current_pit = RESET_ANGLE;
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
        return command_;
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
    FireTask fire_task;     // 开火任务
    ResetTask reset_task;   // 复位任务

    bool filter_flag = false;

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
        readXML();  // 读取调整参数
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

    /**
     * @brief 判断方向
     * @param 输入能量机关角度
     * @return
     */
    int getSimpleDirection(float angle);

    void readXML(){
        FileStorage fs("../rm-vision/buff_detection/buff_offset.xml", FileStorage::READ);
        fs["offset_x"] >> buff_offset_x_;
        fs["offset_y"] >> buff_offset_y_;

        if(buff_offset_x_ <= 0 || buff_offset_x_ >= 200)
            buff_offset_x_ = BUFF_OFFSET_x;
        if(buff_offset_y_ <= 0 || buff_offset_y_ >= 200)
            buff_offset_y_ = BUFF_OFFSET_y;
        begin_offset_x_ = buff_offset_x_;
        begin_offset_y_ = buff_offset_y_;
        cout << "read:"<<buff_offset_x_ << ", " << buff_offset_y_ << endl;
        fs.release();
    }
    void writeXML(){
        FileStorage fs("../rm-vision/buff_detection/buff_offset.xml", FileStorage::WRITE);
        fs << "offset_x" << buff_offset_x_;
        fs << "offset_y" << buff_offset_y_;

        cout << "write:"<<buff_offset_x_ << ", " << buff_offset_y_ << endl;
        fs.release();
    }
private:
    bool DetectBuff(Mat& img, OtherParam other_param);

    // 外部参数
private:
    int color_;
    float gimbal;

    // debug参数
public:
    int buff_offset_x_ = BUFF_OFFSET_x;//id:3 112;// id:2 130;
    int buff_offset_y_ = BUFF_OFFSET_y;//id:3 69;// id:2 135;
    int begin_offset_x_ = BUFF_OFFSET_x;
    int begin_offset_y_ = BUFF_OFFSET_y;


    int world_offset_x_ = WORLD_OFFSET_X;
    int world_offset_y_ = 500;
    int pitch_offset = 2000;
    int color_th_ = COLOR_TH;
    int gray_th_ = 50;
    float buff_angle_ = 0;
    float diff_angle_ = 0;
    int area_ratio_ = 500;

    //相关类申明
    AutoControl auto_control;
private:
    SolveAngle solve_angle_long_;
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
    int last_angle_;
    int find_cnt = 0;
    int direction_tmp=0;

public:
    int waitkey_flag = 1;
    int imshow_flag = 1;


    int command = 0;
};

double Point_distance(Point2f p1,Point2f p2);



