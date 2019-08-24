#pragma once
#include "iostream"
#include "opencv2/opencv.hpp"

#define CANCLE_GALAXY   // 取消工业相机宏定义

// 资源岛测试快速宏定义-<<<----------------
#define CAMERA1_FILEPATH "../rm-vision/camera/camera_param/\
galaxy_0.xml"
//#define SAVE_VIDEO_THREAD
#define WAITKEY 1
#define IMAGESHOW
// 能量机关自动控制项
//#define NO_FIRE   // 发现新目标射一发子弹
//#define NO_REPEAT_FIRE    // 没击打重复发
// 调试状态
// 1:    2:   3:
#define BUFF_OFFSET_x 101// 1:80// 3:112
#define BUFF_OFFSET_y 118// 1:125// 3:69

#define WORLD_OFFSET_X 750
#define COLOR_TH 20

#define FIRE_CNT 30             // 越小响应越快
#define RESET_CNT 30            // 丢失目标复位计数 越小响应越快
#define REPEAT_FIRE_TIME 1000   // 重复发射时间，单位ｍｓ
#define FIRE_LIMIT_ANGLE 2.0f
#define RESET_ANGLE -10 // 1:-20 else: -10  // 复位绝对角度
// 固定状态
#define BULLET_SPEED 28.5
#define BUFF_H 800
#define BUFF_DISTANCE 7300

#define GALAXY_EXPOSURE_TIME 1500
// 资源岛测试快速宏定义-<<<----------------
// ****** 整个系统的调试  ******//
//#define GALAXY
//#define DEBUG_PLOT
//****** 线程使能 *****
#define SHORT_CAMERA_ENABLE 0
#define LONG_CAMERA_ENABLE  0
//#define GET_STM32_THREAD
//****** 装甲板识别配置 *****
#define ARMOR_TRACK_BAR
#define DEBUG_ARMOR_DETECT
#define ROI_ENABLE

//****** 能量机关识别信息 *****
#define BUFF_TRACK_BAR
#define DEBUG_BUFF_DETECT


//****** 摄像头信息 *****
#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 360
#define BUFFER_SIZE 1

//****** 外部驱动配置 *****
#define SERIAL_PATH "/dev/stm32"
#define SERIAL_BAUD B115200     // B115200 B921600
#define GIMBAL_PATH "/dev/ttyUSB0"
#define GIMBAL_BAUD B921600
#define CAMERA0_PATH "/dev/camera"
#define CAMERA1_PATH "/dev/video1"
#define CAMERA0_FILEPATH "../rm-vision/camera/camera_param/\
camera4mm_5.xml"


//****** 角度解算配置 *****
//#define SET_ZEROS_GRAVITY
//#define SIMPLE_SOLVE_ANGLE_FOR_ARMOR_DETECT
// 摄像头坐标系到云台坐标系
#define SHOR_X 57.0f
#define SHOR_Y 47.5f
#define SHOR_Z -111.37f
#define LONG_X 0.0f
#define LONG_Y 40.7f
#define LONG_Z -123.0f
#define PTZ_TO_BARREL 0.0f   // 补兵激光在２３ｍｍ下方

//****** 笔记本调试相关参数 *****
#define DEBUG_VIDEO 1
//#define FORCE_CHANGE_CAMERA
#define ARMOR_VIDEO_PATH "../Videos/test.avi"
#define BUFF_VIDEO_PATH "../Videos/test.avi"
struct OtherParam
{
    int color = 1;       // 我方车辆颜色，0是蓝色，1是红色。用于图像预处理
    int mode = 0;        // 视觉模式，0是自瞄模式，1是能量机关模式
    int cap_mode = 1;    // 摄像头类型，0是短焦摄像头，1是长焦摄像头
    float gimbal_data;
    float buff_offset_x;
    float buff_offset_y;
};

// ****** common ******//
#define END_THREAD if(end_thread_flag) return;
#define INFO(a) cout<<#a<<"="<<a<<endl;
#define TIME_START(a) double a=getTickCount();
#define TIME_END(a) cout<<#a<<" "<<(getTickCount()-a)*1000/getTickFrequency()<<endl;
#define NOTICE(test, num){                   \
    static bool flag = true;            \
    static int i=0; \
    if(flag)                            \
{              \
    i++;                          \
    std::cout << test << std::endl; \
    if(i>=num)               \
    flag = false;                   \
    }                                   \
    }                                       \

#define INFANTRY  0
#define HERO 1
#define PLANE 2

#define B115200 0
#define B921600 1

#define name_string(name) name,#name

class PutTry
{
public:
    PutTry(){}
    ~PutTry(){}
    void TextPut_(cv::Mat &img, double &double_text,const std::string dou_ch) //name_string(a) instead the two
    {
        double self_indexd;

        if(self_index_def-double_text>1)
        {
            self_add_count+=1;
            self_index_def=double_text;
        }
        self_indexd=20*self_add_count;
        cv::Point2d view_all= cv::Point2d(5,self_indexd);
        putText(img, dou_ch + " : "+ std::to_string(double_text),view_all,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255),1);
    }
    void TextPut_(cv::Mat &img, float &float_text,const std::string flo_ch)
    {
        float self_indexf;

        if(self_index_def-float_text>1)
        {
            self_add_count+=1;
            self_index_def=float_text;
        }
        self_indexf=20*self_add_count;
        cv::Point2f view_all = cv::Point2f(5,self_indexf);
        putText(img,flo_ch + " : " + std::to_string(float_text),view_all,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255),1);
    }
    void TextPut_(cv::Mat &img, int &int_text,const std::string int_ch) //name_string(a) instead the two
    {
        double self_indexd;

        if(self_index_def-int_text>1)
        {
            self_add_count+=1;
            self_index_def=int_text;
        }
        self_indexd=20*self_add_count;
        cv::Point2d view_all= cv::Point2d(5,self_indexd);
        putText(img, int_ch + " : "+ std::to_string(int_text),view_all,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255),1);
    }
private:
    int self_add_count=1;

    double self_index_def=1e8;

};


//#define TIMER_START boost::timer t_##__func__;
//#define TIMER_END std:: cout << "[" << #__func__ << "]" << "cost time: " << t_##__func__.elapsed() << std::endl;



//                printf("debug test: armor_type = %d, bullet_speed = %f", final_armor_type, bullet_speed_);
//                putText(image, "origin z :" + to_string(distance) + " x: " + to_string(angle_x) + " y :" + to_string(angle_y) + "theta_y :" + to_string(theta_y)
//                        , Point(0,20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
//                putText(image, "orICRA z :" + to_string(distance_i) + " x: " + to_string(angle_x_i) + " y :" + to_string(angle_y_i)
//                        , Point(0,40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));



//    ofstream file("test.txt");
//        file << csmIdx << " " << angle_x << " " << angle_y << "\n";
//    file.close();
