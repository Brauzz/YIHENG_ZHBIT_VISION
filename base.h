#pragma once
#include "iostream"


#define ROBOT_TYPE INFANTRY // INFANTRY HERO PLANE

// ****** systems  ******//
//#define SHORT_CAMERA_ENABLE 1
#define LONG_CAMERA_ENABLE  1
//#define GET_STM32_THREAD
//#define GET_GIMBAL_THREAD
#define WAITKEY 1
#define IMAGESHOW
// ****** settings ******//
#define GALAXY
#define DEBUG_PLOT

// for armor --------------
#define DEBUG_ARMOR_DETECT
#define ROI_ENABLE
//#define PREDICT
// for buff --------------
#define DEBUG_BUFF_DETECT

// for image
#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 360
#define BUFFER_SIZE 1

// device setting
#define SERIAL_PATH "/dev/stm32"
#define SERIAL_BAUD B115200     // B115200 B921600
#define GIMBAL_PATH "/dev/ttyUSB1"
#define GIMBAL_BAUD B921600
#define CAMERA0_PATH "/dev/camera"
#define CAMERA1_PATH "/dev/video1"

#define CAMERA0_FILEPATH "../rm-vision/camera/camera_param/\
camera4mm_5.xml"
#define CAMERA1_FILEPATH "../rm-vision/camera/camera_param/\
galaxy_1.xml"

#define DEBUG_VIDEO 1
#define FORCE_CHANGE_CAMERA
struct OtherParam
{
    int8_t color = 1;       // 我方车辆颜色，0是蓝色，1是红色。用于图像预处理
    int8_t mode = 1;        // 视觉模式，0是自瞄模式，1是能量机关模式
    int8_t cap_mode = 1;    // 摄像头类型，0是短焦摄像头，1是长焦摄像头
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
