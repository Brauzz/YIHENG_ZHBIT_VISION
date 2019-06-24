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

static volatile bool end_thread_flag  = false;  // 设置结束线程标志位，负责结束所有线程。
static volatile unsigned int produce_index;     // 图像生成序号，用于线程之间逻辑
static volatile unsigned int gimbal_data_index;     // gimbal data 生成序号，用于线程之间逻辑
static volatile unsigned int consumption_index; // 图像消耗序号
static volatile bool cap_mode_ = 1;             // 初始时摄像头模式为0，短焦
//static volatile float new_gimbal_yaw = 0.0f;
int8_t mode_ = 0, color_ = 0;

static ImageData data[BUFFER_SIZE];
static OtherParam other_param;
SerialPort serial_("/dev/ttyUSB1",0);                 // pc与stm32之间的串口通信
SerialPort serial_gimbal_("/dev/ttyUSB0",1);          // pc与陀螺仪之间的串口通信

ThreadControl::ThreadControl()
{
    cout << "THREAD TASK ON !!!" << endl;
    other_param.color = 0;
    other_param.mode = 0;
    other_param.cap_mode = 0;
    mode_ = false;
    cap_mode_ = false;
}

// 图像生成线程
void ThreadControl::ImageProduce()
{
    cout << " ------ SHORT CAMERA PRODUCE TASK ON !!! ------ " << endl;
    CaptureVideo short_camera("/dev/camera", 3);                // 选择相机驱动文件，可在终端下输入"ls /dev" 查看. 4帧缓存
        short_camera.setVideoFormat(VIDEO_WIDTH, VIDEO_HEIGHT, 1);   // 设置长宽格式及使用mjpg编码格式
        short_camera.setExposureTime(0, 100);                         // 手动曝光，设置曝光时间。
        short_camera.startStream();                                  // 打开视频流
//        short_camera.info();                                         // 输出摄像头信息
    while(1)
    {
        // 等待图像进入处理瞬间，再去生成图像
        while(produce_index - consumption_index >= BUFFER_SIZE || cap_mode_ == true)
            END_THREAD;
        short_camera >> data[produce_index % BUFFER_SIZE].img;
        if(short_camera.getFD() != -1)
            ++produce_index;
        END_THREAD;
    }
}

// 图像生成线程
void ThreadControl::ImageProduceLong()
{
    cout << " ------LONG CAMERA PRODUCE TASK ON !!! ------" << endl;
#ifdef GALAXY
    // 工业相机类初始化
    CameraDevice galaxy;
    galaxy.init();
    while(1)
    {
        while(produce_index - consumption_index >= BUFFER_SIZE || cap_mode_ == false)
            END_THREAD;
        galaxy.getImage(data[produce_index % BUFFER_SIZE].img);
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

void ThreadControl::GetGimbal()
{
    cout << " ------GIMBAL DATA RECEVICE TASK ON !!! ------ " << endl;
    GimbalDataProcess GimDataPro;
    serial_gimbal_data gim_rx_data_;    // 陀螺仪接受数据格式
    float raw_gimbal_yaw, dst_gimbal_yaw;
    while(1)
    {
        while(static_cast<int>(gimbal_data_index - consumption_index) >= BUFFER_SIZE)
        {
            END_THREAD;
        }

        serial_gimbal_.read_gimbal(&gim_rx_data_, raw_gimbal_yaw);
        if(serial_gimbal_.success_)
        {
            GimDataPro.ProcessGimbalData(raw_gimbal_yaw, dst_gimbal_yaw);
            data[consumption_index % BUFFER_SIZE].gimbal_data = dst_gimbal_yaw;
        }
        gimbal_data_index++;
        END_THREAD;
    }
}

void ThreadControl::GetSTM32()
{
    cout << " ------ STM32 DATA RECEVICE TASK ON !!! ------" << endl;
    serial_receive_data rx_data;        // 串口接收stm32数据结构
    while(1){
        //            int8_t modae;
        // 每50帧读取一次stm32数据

        if(produce_index%50 == 0)
        {
            serial_.read_data(&rx_data, mode_, color_, bullet_speed_, cancel_kalman_);
{
//                                    if(mode_ == 0)
//                                        cout << " mode=auto_aim  ";
//                                    else
//                                        cout << " mode=buff_aim  ";
//                                    if(color_ == 0)
//                                        cout << "  BLUE  ";
//                                    else
//                                        cout << "  RED  ";
//                        //            cout << "bullet_speed = " << bullet_speed_;
//                                    if(cancel_kalman_ == 0)
//                                        cout << "  use_kalman "<<endl;
//                                    else
//                                        cout << "  cancel_kalman "<<endl;
//                        INFO(bullet_speed_);
}
        }
//        waitKey(1);
        END_THREAD;
    }
}

// 图像处理线程
void ThreadControl::ImageProcess()
{
    cout << " ------ IMAGE PROCESS TASK ON !!! ------" << endl;
    Parameter para_armor;
    {
        namedWindow("control");
        createTrackbar("gray_th", "control", &para_armor.gray_th, 255);
        createTrackbar("color_th", "control", &para_armor.color_th, 255);
        createTrackbar("short_offset_x","control",&para_armor.short_offset_x,200);
        createTrackbar("short_offset_y","control",&para_armor.short_offset_y,200);
        createTrackbar("long_offset_x","control",&para_armor.long_offset_x,200);
        createTrackbar("long_offset_y","control",&para_armor.long_offset_y,200);
        createTrackbar("world_offset_x","control",&para_armor.world_offset_x,1000);
        createTrackbar("world_offset_y","control",&para_armor.world_offset_y,1000);
        createTrackbar("Qp","control",&km_Qp,1000);
        createTrackbar("Qv","control",&km_Qv,1000);
        createTrackbar("rp","control",&km_Rp,1000);
        createTrackbar("rv","control",&km_Rv,1000);
        createTrackbar("t","control",&km_t,10);
        createTrackbar("pt","control",&km_pt,500);
    }

    Mat image;
    float new_gimbal_data = 0.0;
    float angle_x = 0.0, angle_y = 0.0, distance = 3000.0, theta_y = 0.0;
    float dist = 3000.0f, r = 0.8f;
    float predict_angle_x = 0.0;
    serial_transmit_data tx_data;       // 串口发送stm32数据结构
    ArmorFilter armor_filter(5);
    DirectionFilter direction_filter(20, 15);
    bool find_flag = false;
    int key = 0;
    // 角度结算类声明
    SolveAngle solve_angle("/home/cz/rm-vision/camera4mm.xml", -20, 80, -135, 0);
    SolveAngle solve_angle_long("/home/cz/rm-vision/galaxy_0.xml", 0, 40.0, -135, 0);
    // 预测类声明
    ZeYuPredict zeyu_predict(0.01f, 0.01f, 0.01f, 0.01f, 1.0f, 3.0f);
    //    ofstream file("test.txt");
    while(1)
    {
        // 等待图像生成后进行处理
        while(produce_index - consumption_index == 0){}
        data[consumption_index % BUFFER_SIZE].img.copyTo(image);
        new_gimbal_data = data[consumption_index % BUFFER_SIZE].gimbal_data;
        para_armor.cap_mode_ = cap_mode_;
        para_armor.color = color_;
//        NOTICE("coming~",3);
        if(1)
        {
            //***************************auto_mode***********************************
            //***************************auto_mode***********************************
            dist = (1-r)*dist + r * distance;
            if(dist > 1500)
            {
                cap_mode_ = true;
            }
            else if(dist < 1000)
            {
                cap_mode_ = false;
            }
//                        if(key == 'c')
//                            cap_mode_ = !cap_mode_;
//            cout << cap_mode_ << endl;

            ++consumption_index;
            double t1 = getTickCount();
            find_flag = ArmorDetectTask(image, para_armor);   // 装甲板检测
            double t2 = getTickCount();
            double t = (t2 - t1)*1000/getTickFrequency();

            bool final_armor_type = armor_filter.getResult(para_armor.is_small);
            if(find_flag)
            {
                if(cap_mode_ == false) // close
                {
                    solve_angle.Generate3DPoints((uint8_t)final_armor_type, Point2f());
                    solve_angle.getAngle(para_armor.points_2d, 15,angle_x,angle_y,distance,theta_y);   // pnp姿态结算
                }
                else                    // far
                {
                    solve_angle_long.Generate3DPoints((uint8_t)final_armor_type, Point2f());
                    solve_angle_long.getAngle(para_armor.points_2d, 15,angle_x,angle_y,distance,theta_y);   // pnp姿态结算
                }
                //                printf("debug test: armor_type = %d, bullet_speed = %f", final_armor_type, bullet_speed_);
                //                putText(image, "origin z :" + to_string(distance) + " x: " + to_string(angle_x) + " y :" + to_string(angle_y) + "theta_y :" + to_string(theta_y)
                //                        , Point(0,20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
                //                putText(image, "orICRA z :" + to_string(distance_i) + " x: " + to_string(angle_x_i) + " y :" + to_string(angle_y_i)
                //                        , Point(0,40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));

                //---predict
                protectDate(km_Qp, km_Qv, km_Rp, km_Rv, km_t, km_pt);
                float pre_time = distance/10000*static_cast<float>(km_pt)+10.0f;
                zeyu_predict.setQRT(km_Qp,km_Qv,km_Rp,km_t,pre_time);
                if(serial_gimbal_.success_)   // gimbal is successed
                {
                    float gim_and_pnp_angle_x = -new_gimbal_data + angle_x;
                    predict_angle_x = zeyu_predict.run_position(gim_and_pnp_angle_x);   // kalman滤波预测
                    predict_angle_x += new_gimbal_data;
                    angle_x = predict_angle_x;
                }
            }

        }
        else
        {
            //***************************buff_mode***********************************
            //***************************buff_mode***********************************

            ++consumption_index;
            para_armor.cap_mode_ = true;
            find_flag = BuffDetectTask(image, para_armor);
            if(find_flag)
            {
                bool direction_tmp = direction_filter.getDirection(para_armor.buff_angle);
                //                cout << buff_angle << "  " << direction_tmp << endl;
                Point2f world_offset;
                //#define DIRECTION_FILTER
#ifdef DIRECTION_FILTER
                if(direction_tmp == 1)  // shun
                    world_offset = Point2f(para_armor.world_offset_x  - 500, para_armor.world_offset_y - 500);
                else // ni
                    world_offset = Point2f(-(para_armor.world_offset_x  - 500), -(para_armor.world_offset_y - 500));
                cout << "direction " << direction_tmp << endl;
#else
                world_offset = Point2f(para_armor.world_offset_x - 500, para_armor.world_offset_y  - 500);
#endif

                solve_angle_long.Generate3DPoints(2, world_offset);
                solve_angle_long.getAngle(para_armor.points_2d, 28, angle_x, angle_y, distance, theta_y);
            }
        }
        //--/predict
        limit_angle(angle_x, 3);
        //        file << csmIdx << " " << angle_x << " " << angle_y << "\n";
        tx_data.get_xy_data(-angle_x*100, -angle_y*100,find_flag);
        serial_.send_data(tx_data);
//        imshow("image",image);
//        key = waitKey(1);
//        if(key == 'q')
//            end_thread_flag = true;
        //        //                        usleep(1);
        END_THREAD;
    }
    //    file.close();
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
