#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include "thread"
#include "unistd.h"
#include "chrono"
#include "capture_video.h"
#include "armor_detect.h"
#include "buff_detect.h"
#include "serial_port.h"
#include "solve_angle.h"
#include "predict.h"
#include "settings.h"

#define END_THREAD if(end_thread_flag) return;
#define INFO(a) cout<<#a<<a<<endl;
#define TIME_START(a) double a=getTickCount();
#define TIME_END(a) cout<<#a<<" "<<(getTickCount()-a)*1000/getTickFrequency()<<endl;

#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 360
#define BUFFER_SIZE 1

using namespace cv;
using namespace std;

class ImageProduceProcess
{

public:
    ImageProduceProcess();
    void ImageProduce();
    void ImageProcess();
    void GetGimbal();
    void GetSTM32();
public:

    int8_t mode_;
    int8_t color_;
    float bullet_speed_;
    int8_t cancel_kalman_;
    // kalman predict
    int km_Qp = 1000, km_Qv = 1, km_Rp = 1, km_Rv = 1;
    int km_t = 4, km_pt = 70;
    int history_index = 1;
private:
    serial_gimbal_data gim_rx_data_;
    serial_transmit_data tx_data;
    serial_receive_data rx_data;
    SerialPort serial_;
    SerialPort serial_gimbal_;
};

void protectDate(int& a, int &b, int &c, int& d, int& e, int& f);
void limit_angle(float &a, float max);
