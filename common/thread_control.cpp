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
#include "thread_control.h"

//static volatile bool end_thread_flag  = false;  // 设置结束线程标志位，负责结束所有线程。
static volatile unsigned int produce_index;     // 图像生成序号，用于线程之间逻辑
static volatile unsigned int gimbal_data_index;     // gimbal data 生成序号，用于线程之间逻辑
static volatile unsigned int consumption_index; // 图像消耗序号

#ifdef GET_STM32_THREAD
SerialPort serial_("/dev/ttyUSB0",0);                 // pc与stm32之间的串口通信
#endif
#ifdef GET_GIMBAL_THREAD
SerialPort serial_gimbal_("/dev/ttyUSB0",1);          // pc与陀螺仪之间的串口通信
#endif

ThreadControl::ThreadControl()
{
    cout << "THREAD TASK ON !!!" << endl;
}

// 图像生成线程
void ThreadControl::ImageProduce()
{
    cout << " ------ SHORT CAMERA PRODUCE TASK ON !!! ------ " << endl;
    camera0_enable = true;
    CaptureVideo short_camera("/dev/video1", 3);                // 选择相机驱动文件，可在终端下输入"ls /dev" 查看. 4帧缓存
    short_camera.setVideoFormat(VIDEO_WIDTH, VIDEO_HEIGHT, 1);   // 设置长宽格式及使用mjpg编码格式
    short_camera.setExposureTime(0, 100);                         // 手动曝光，设置曝光时间。
    short_camera.startStream();                                  // 打开视频流
    //        short_camera.info();                                         // 输出摄像头信息

    while(1)
    {
        // 等待图像进入处理瞬间，再去生成图像
        while(!(produce_index - consumption_index <= 0
              && (other_param.cap_mode == 0 || (other_param.cap_mode == 1 && camera1_enable == 0))))
            END_THREAD;
        short_camera >> image_;
        if(short_camera.getFD() != -1)
            ++produce_index;
        END_THREAD;
    }
}

// 图像生成线程
void ThreadControl::ImageProduceLong()
{
    cout << " ------LONG CAMERA PRODUCE TASK ON !!! ------" << endl;
    camera1_enable = true;
#ifdef GALAXY
    // 工业相机类初始化
    CameraDevice galaxy;
    galaxy.init();

    while(1)
    {
        while(!(produce_index - consumption_index <= 0
               &&(other_param.cap_mode == 1 || (other_param.cap_mode == 0 && camera0_enable == 0))))
            END_THREAD;
        galaxy.getImage(image_);
        ++produce_index;
        END_THREAD;
    }

#else
    CaptureVideo long_camera("/dev/video2", 3);                // 选择相机驱动文件，可在终端下输入"ls /dev" 查看. 4帧缓存
    long_camera.setVideoFormat(VIDEO_WIDTH, VIDEO_HEIGHT, 1);   // 设置长宽格式及使用mjpg编码格式
    long_camera.setExposureTime(0, 20);                         // 手动曝光，设置曝光时间。
    long_camera.startStream();                                  // 打开视频流
    long_camera.info();                                         // 输出摄像头信息
    while(1)
    {
        while(prdIdx - csmIdx >= BUFFER_SIZE || cap_mode_ == false)
            END_THREAD;
        long_camera >> data[prdIdx % BUFFER_SIZE].img;
        ++prdIdx;
        END_THREAD;
    }
#endif
}

#ifdef GET_GIMBAL_THREAD
void ThreadControl::GetGimbal()
{
    cout << " ------GIMBAL DATA RECEVICE TASK ON !!! ------ " << endl;
    GimbalDataProcess GimDataPro;
    serial_gimbal_data gim_rx_data_;    // 陀螺仪接受数据格式
    float raw_gimbal_yaw, dst_gimbal_yaw;
    while(1)
    {
        while(static_cast<int>(gimbal_data_index - consumption_index) >= BUFFER_SIZE)
            END_THREAD;
        serial_gimbal_.read_gimbal(&gim_rx_data_, raw_gimbal_yaw);

        data[consumption_index % BUFFER_SIZE].serial_success = serial_gimbal_.success_;
        if(serial_gimbal_.success_)
        {
            GimDataPro.ProcessGimbalData(raw_gimbal_yaw, dst_gimbal_yaw);
            INFO(dst_gimbal_yaw);
            data[consumption_index % BUFFER_SIZE].gimbal_data = dst_gimbal_yaw;
        }
        gimbal_data_index++;
        END_THREAD;
    }
}
#endif

#ifdef GET_STM32_THREAD
void ThreadControl::GetSTM32()
{
    cout << " ------ STM32 DATA RECEVICE TASK ON !!! ------" << endl;
    serial_receive_data rx_data;        // 串口接收stm32数据结构
    GimbalDataProcess GimDataPro;
    float raw_gimbal_yaw, dst_gimbal_yaw;
    int8_t mode = 0;
    int8_t color = 0;
    while(1){
        while(static_cast<int>(gimbal_data_index - consumption_index) >= BUFFER_SIZE)
            END_THREAD;

        serial_.read_data(&rx_data, mode, color, raw_gimbal_yaw);
        GimDataPro.ProcessGimbalData(raw_gimbal_yaw, dst_gimbal_yaw);
        data[consumption_index % BUFFER_SIZE].gimbal_data = dst_gimbal_yaw;
        INFO(dst_gimbal_yaw);
        gimbal_data_index++;
        END_THREAD;
    }
}
#endif

// 图像处理线程
void ThreadControl::ImageProcess()
{
    cout << " ------ IMAGE PROCESS TASK ON !!! ------" << endl;
    // 角度结算类声明
    SolveAngle solve_angle("../rm-vision/camera/camera_param/camera4mm.xml", -20, 80, -135, 0);
    SolveAngle solve_angle_long("../rm-vision/camera/camera_param/galaxy_0.xml", 0, 40.0, -135, 0);
    // 预测类声明
    ZeYuPredict zeyu_predict(0.01f, 0.01f, 0.01f, 0.01f, 1.0f, 3.0f);
    ArmorDetector armor_detector(solve_angle, solve_angle_long, zeyu_predict);
    BuffDetector buff_detector(solve_angle_long);
    serial_transmit_data tx_data;       // 串口发送stm32数据结构
    {
        namedWindow("ArmorParam");
        namedWindow("BuffParam");
        createTrackbar("armor_gray_th", "ArmorParam", &armor_detector.gray_th_, 255);
        createTrackbar("armor_color_th", "ArmorParam", &armor_detector.color_th_, 255);
        createTrackbar("short_offset_x","ArmorParam",&armor_detector.short_offset_x_,200);
        createTrackbar("short_offset_y","ArmorParam",&armor_detector.short_offset_y_,200);
        createTrackbar("long_offset_x","ArmorParam",&armor_detector.long_offset_x_,200);
        createTrackbar("long_offset_y","ArmorParam",&armor_detector.long_offset_y_,200);
        createTrackbar("Qp","ArmorParam",&armor_detector.km_Qp_,1000);
        createTrackbar("Qv","ArmorParam",&armor_detector.km_Qv_,1000);
        createTrackbar("rp","ArmorParam",&armor_detector.km_Rp_,1000);
        createTrackbar("rv","ArmorParam",&armor_detector.km_Rv_,1000);
        createTrackbar("t","ArmorParam",&armor_detector.km_t_,10);
        createTrackbar("pt","ArmorParam",&armor_detector.km_pt_,500);
        createTrackbar("buff_gray_th", "BuffParam", &buff_detector.gray_th_, 255);
        createTrackbar("buff_color_th", "BuffParam", &buff_detector.color_th_, 255);
        createTrackbar("buff_offset_x_","BuffParam",&buff_detector.buff_offset_x_,200);
        createTrackbar("buff_offset_y_","BuffParam",&buff_detector.buff_offset_y_,200);
        createTrackbar("world_offset_x","BuffParam",&buff_detector.world_offset_x_,1000);
        createTrackbar("world_offset_y","BuffParam",&buff_detector.world_offset_y_,1000);
    }
    Mat image;
    float angle_x = 0.0, angle_y = 0.0;
    int8_t find_flag = 0;
    while(1)
    {
        // 等待图像生成后进行处理
        while(produce_index - consumption_index <= 0){
            END_THREAD;
        }
        // 数据初始化
        image_.copyTo(image);

        if(other_param.mode == 0)
        {
            //            ***************************auto_mode***********************************
#ifndef FORCE_CHANGE_CAMERA
            other_param.cap_mode = armor_detector.chooseCamera(1000, 1500, other_param.cap_mode);
#endif
            ++consumption_index;

            double t1 = getTickCount();
            find_flag = armor_detector.ArmorDetectTask(image, other_param);
            double t2 = getTickCount();
            double t = (t2 - t1)*1000/getTickFrequency();
//                INFO(t);
            armor_detector.getAngle(angle_x, angle_y);
        }
        else
        {
            //***************************buff_mode***********************************
            other_param.cap_mode = 1;
            ++consumption_index;

            find_flag = buff_detector.BuffDetectTask(image, other_param);
            if(find_flag)
                buff_detector.getAngle(angle_x, angle_y);
        }
        limit_angle(angle_x, 5);
#ifdef GET_STM32_THREAD
        tx_data.get_xy_data(-angle_x*300, -angle_y*100,find_flag);
        serial_.send_data(tx_data);
#endif
#ifdef WAITKEY
#ifdef IMAGESHOW
        imshow("image", image);
#endif
        char key = waitKey(1);
        if(key == 'q')
            end_thread_flag = true;
        END_THREAD;
#endif
    }
}

void protectDate(int& a, int &b, int &c, int& d, int& e, int& f)
{
    if(a<=0)
        a = 1;
    if(b<=0)
        b = 1;
    if(c<=0)
        c = 1;
    if(d<=0)
        d = 1;
    if(e<=0)
        e = 1;
    if(f<=0)
        f = 1;
}

void limit_angle(float &a, float max)
{
    if(a > max)
        a = max;
    else if(a < -max)
        a = -max;
}
