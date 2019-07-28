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
static volatile unsigned int save_image_index;  // 保存图像序号

#ifdef GET_STM32_THREAD
SerialPort serial_(SERIAL_PATH,SERIAL_BAUD);                 // pc与stm32之间的串口通信
#endif
#ifdef GET_GIMBAL_THREAD
SerialPort serial_gimbal_(GIMBAL_PATH,GIMBAL_BAUD);          // pc与陀螺仪之间的串口通信
#endif

ThreadControl::ThreadControl()
{
    cout << "THREAD TASK ON !!!" << endl;
}

// 图像生成线程
void ThreadControl::ImageProduce()
{
    cout << " ------ SHORT CAMERA PRODUCE TASK ON !!! ------ " << endl;


#if(SHORT_CAMERA_ENABLE)
    CaptureVideo short_camera(CAMERA0_PATH, 3);                // 选择相机驱动文件，可在终端下输入"ls /dev" 查看. 4帧缓存
    short_camera.setVideoFormat(VIDEO_WIDTH, VIDEO_HEIGHT, 1);   // 设置长宽格式及使用mjpg编码格式
    short_camera.setExposureTime(0, 100);                         // 手动曝光，设置曝光时间。
    short_camera.startStream();                                  // 打开视频流
    //        short_camera.info();                                         // 输出摄像头信息
#endif
#if(LONG_CAMERA_ENABLE)
#ifdef GALAXY
    CameraDevice galaxy;
    galaxy.init();
#else
    CaptureVideo long_camera(CAMERA1_PATH, 3);                // 选择相机驱动文件，可在终端下输入"ls /dev" 查看. 4帧缓存
    long_camera.setVideoFormat(VIDEO_WIDTH, VIDEO_HEIGHT, 1);   // 设置长宽格式及使用mjpg编码格式
    long_camera.setExposureTime(0, 100);                         // 手动曝光，设置曝光时间。
    long_camera.startStream();                                  // 打开视频流
#endif
#endif
    while(1)
    {
        // 等待图像进入处理瞬间，再去生成图像
        while(produce_index - consumption_index >= BUFFER_SIZE)
            END_THREAD;
#if(SHORT_CAMERA_ENABLE==1&&LONG_CAMERA_ENABLE==1)
        if(other_param.cap_mode == 0)
#endif
#if(SHORT_CAMERA_ENABLE)
            short_camera >> image_;
#endif
#if(SHORT_CAMERA_ENABLE==1&&LONG_CAMERA_ENABLE==1)
        else if(other_param.cap_mode == 1)
#endif
#if(LONG_CAMERA_ENABLE)
#ifdef GALAXY
            galaxy.getImage(image_);
#else
            long_camera >> image_;
#endif
#endif
        ++produce_index;
        END_THREAD;
    }
}


#ifdef GET_GIMBAL_THREAD
void ThreadControl::GetGimbal() //give up
{
    cout << " ------GIMBAL DATA RECEVICE TASK ON !!! ------ " << endl;
    GimbalDataProcess GimDataPro;
    serial_gimbal_data gim_rx_data_;    // 陀螺仪接受数据格式
    float raw_gimbal_yaw, dst_gimbal_yaw = 0.0;
    Predictor predictor(20);
    double t_start = getTickCount();
    double t_tmp = t_start;
    float predict = 0.0;
    while(1)
    {
        while(static_cast<int>(gimbal_data_index - consumption_index) >= BUFFER_SIZE)
            END_THREAD;
        serial_gimbal_.read_gimbal(&gim_rx_data_, raw_gimbal_yaw);
        if(serial_gimbal_.success_)
        {
            GimDataPro.ProcessGimbalData(raw_gimbal_yaw, dst_gimbal_yaw);
            float gimbal_data = dst_gimbal_yaw;
            t_tmp = getTickCount();
            predictor.setRecord(gimbal_data, (t_tmp - t_start)*1000/getTickFrequency());
            predict = predictor.predict(((t_tmp - t_start)*1000/getTickFrequency())+100);
        }

        gimbal_data_index++;
#ifdef DEBUG_PLOT
        if(debug_enable_flag == true)
        {
            w_->addPoint(dst_gimbal_yaw,0);
            w_->addPoint(predict, 1);
            w_->plot();
        }
#endif
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
    bool mode = 0;
    bool color = 0;
    int cnt=0;
    while(1){
        while(static_cast<int>(gimbal_data_index - consumption_index) >= BUFFER_SIZE)
            END_THREAD;

        serial_.read_data(&rx_data, mode, color, raw_gimbal_yaw);
        other_param.mode = mode;
        other_param.color = color;
        GimDataPro.ProcessGimbalData(raw_gimbal_yaw, dst_gimbal_yaw);
        float gimbal_data = dst_gimbal_yaw;
        other_param.gimbal_data = gimbal_data;

        if((gimbal_data_index%50)==0)
        {
            printf("Id: %d, Mode: %d, Color: %d\r\n", gimbal_data_index, mode, color);
        }

#ifdef DEBUG_PLOT
        if(debug_enable_flag == true)
        {
            w_->addPoint(cnt,0);
            w_->plot();
        }
#endif
        gimbal_data_index++;
        END_THREAD;
    }
}
#endif

// 图像处理线程
void ThreadControl::ImageProcess()
{
    cout << " ------ IMAGE PROCESS TASK ON !!! ------" << endl;
    ArmorDetector armor_detector;
#ifdef DEBUG_PLOT
    int argc;char **argv = nullptr;
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    w_ = &w;
    debug_enable_flag = true;
    armor_detector.DebugPlotInit(&w);

#endif

#if(ROBOT_TYPE == INFANTRY)
    BuffDetector buff_detector;
#ifdef DEBUG_PLOT
    buff_detector.DebugPlotInit(&w);
#endif
#endif
    serial_transmit_data tx_data;       // 串口发送stm32数据结构

    namedWindow("ArmorParam");
    createTrackbar("armor_gray_th", "ArmorParam", &armor_detector.gray_th_, 255);
    createTrackbar("armor_color_th", "ArmorParam", &armor_detector.color_th_, 255);
    createTrackbar("short_offset_x","ArmorParam",&armor_detector.short_offset_x_,200);
    createTrackbar("short_offset_y","ArmorParam",&armor_detector.short_offset_y_,200);
    createTrackbar("long_offset_x","ArmorParam",&armor_detector.long_offset_x_,200);
    createTrackbar("long_offset_y","ArmorParam",&armor_detector.long_offset_y_,200);
#ifdef PREDICT
    createTrackbar("Qp","ArmorParam",&armor_detector.km_Qp_,1000);
    createTrackbar("Qv","ArmorParam",&armor_detector.km_Qv_,1000);
    createTrackbar("rp","ArmorParam",&armor_detector.km_Rp_,1000);
    createTrackbar("rv","ArmorParam",&armor_detector.km_Rv_,1000);
    createTrackbar("t","ArmorParam",&armor_detector.km_t_,10);
    createTrackbar("pt","ArmorParam",&armor_detector.km_pt_,500);
#endif
#if(ROBOT_TYPE == INFANTRY)
    namedWindow("BuffParam");
    createTrackbar("buff_gray_th", "BuffParam", &buff_detector.gray_th_, 255);
    createTrackbar("buff_color_th", "BuffParam", &buff_detector.color_th_, 255);
    createTrackbar("buff_offset_x_","BuffParam",&buff_detector.buff_offset_x_,200);
    createTrackbar("buff_offset_y_","BuffParam",&buff_detector.buff_offset_y_,200);
    createTrackbar("world_offset_x","BuffParam",&buff_detector.world_offset_x_,1000);
    createTrackbar("world_offset_y","BuffParam",&buff_detector.world_offset_y_,1000);
#endif
#ifdef DEBUG_VIDEO
#if(DEBUG_VIDEO == 0)
    VideoCapture cap("../Videos/test.avi");
#else
    VideoCapture cap("../Videos/successed.avi");
#endif
#endif

    Mat image;
    float angle_x = 0.0, angle_y = 0.0, distance =  0.0;
    int command = 0;
    while(1)
    {
#ifndef DEBUG_VIDEO
        // 等待图像生成后进行处理
        while(produce_index - consumption_index <= 0){
            END_THREAD;
        }
        // 数据初始化
        image_.copyTo(image);//        image_.copyTo(image);
#else
        cap.read(image);
#endif
#if(ROBOT_TYPE == INFANTRY)
        if(other_param.mode == 0)
        {
            //            ***************************auto_mode***********************************
#ifndef FORCE_CHANGE_CAMERA
            other_param.cap_mode = armor_detector.chooseCamera(1300, 1600, other_param.cap_mode);
#endif
            ++consumption_index;
            //            TIME_START(t);
            command = armor_detector.ArmorDetectTask(image, other_param);
            //            TIME_END(t);
            armor_detector.getAngle(angle_x, angle_y);
        }
        else
        {
            //***************************buff_mode***********************************
#ifndef FORCE_CHANGE_CAMERA
            other_param.cap_mode = 1;
#endif
            ++consumption_index;

            command = buff_detector.BuffDetectTask(image, other_param);
            if(command)
            {
                buff_detector.getAngle(angle_x, angle_y);
                distance = buff_detector.getDistance();
            }

        }

#elif(ROBOT_TYPE == HERO)
#ifndef FORCE_CHANGE_CAMERA
        other_param.cap_mode = armor_detector.chooseCamera(1000, 1500, other_param.cap_mode);
#endif
        ++consumption_index;
        find_flag = armor_detector.ArmorDetectTask(image, other_param);
        armor_detector.getAngle(angle_x, angle_y);


#elif(ROBOT_TYPE == PLANE)
        ++consumption_index;
        find_flag = armor_detector.ArmorDetectTask(image, other_param);
        armor_detector.getAngle(angle_x, angle_y);
#endif

        limit_angle(angle_x, 90);
#ifdef GET_STM32_THREAD
        tx_data.get_xy_data(-angle_x*32767/90, -angle_y*32767/90,find_flag);
        serial_.send_data(tx_data);
#endif

#ifdef WAITKEY
#ifdef IMAGESHOW
        imshow("image", image);
#endif
        char key = waitKey(WAITKEY);
        if(key == 'q')
            end_thread_flag = true;
        if(key == 'c')
        {
            if(other_param.cap_mode == 0)
                other_param.cap_mode = 1;
            else
                other_param.cap_mode = 0;
        }
        END_THREAD;
#endif
    }
}

#ifdef SAVE_VIDEO_THREAD
void ThreadControl::ImageWrite()
{
    cout << " ------ IMAGE WRITE TASK ON !!! ------" << endl;
    SaveVideo writer;
    INFO(writer.getState());
    while(writer.getState()){
        while(static_cast<int>(produce_index - save_image_index) <= 0){
            END_THREAD;
        }
        Mat img_tmp;
        image_.copyTo(img_tmp);
        if(img_tmp.rows == 360)
            copyMakeBorder(img_tmp, img_tmp, 0, 120, 0, 0, BORDER_CONSTANT, Scalar::all(0));
        writer.updateImage(img_tmp);
        save_image_index++;
    }
}
#endif



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
