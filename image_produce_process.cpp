#include "image_produce_process.h"

struct ImageData
{
    Mat img;
    unsigned int frame;
};
//设置结束线程标志位，负责结束所有线程。
volatile bool end_thread_flag  = false;
volatile unsigned int prdIdx;
volatile unsigned int csmIdx;
vector<float> vec_gimbal_yaw;
ImageData data[BUFFER_SIZE];

ImageProduceProcess::ImageProduceProcess()
{
    cout <<"Open the thread of ImageDate" << endl;

    SerialPort serial("/dev/ttyUSB1",0);
    SerialPort serial_gimbal("/dev/ttyUSB0", 1);
    serial_ = serial;
    serial_gimbal_ = serial_gimbal;
}

// 图像生成线程
void ImageProduceProcess::ImageProduce()
{

    CaptureVideo short_camera("/dev/video1", 3);                // 选择相机驱动文件，可在终端下输入"ls /dev" 查看. 4帧缓存
    short_camera.setVideoFormat(VIDEO_WIDTH, VIDEO_HEIGHT, 1);   // 设置长宽格式及使用mjpg编码格式
    short_camera.setExposureTime(0, 20);                         // 手动曝光，设置曝光时间。
    short_camera.startStream();                                  // 打开视频流
    short_camera.info();                                         // 输出摄像头信息
    while(1)
    {
        while(prdIdx - csmIdx >= BUFFER_SIZE)
            END_THREAD;
        short_camera >> data[prdIdx % BUFFER_SIZE].img;
        ++prdIdx;
        END_THREAD;
    }
}

void ImageProduceProcess::GetGimbal()
{
    cout << "receivec task open !" << endl;
    float last_gimbal_yaw = 0.0, curr_gimbal_yaw = 0.0, gimbal_yaw = 0.0;
    int gim_count = 0;
    last_gimbal_yaw = curr_gimbal_yaw;

    while(1)
    {
//        serial_gimbal_.success_ = serial_gimbal_.read_gimbal(&gim_rx_data_, curr_gimbal_yaw);

        if(serial_gimbal_.success_)
        {
            if(curr_gimbal_yaw - last_gimbal_yaw > 270)
            {
                gim_count --;
            }else if(curr_gimbal_yaw - last_gimbal_yaw < -270)
            {
                gim_count++;
            }
            gimbal_yaw= gim_count*360+curr_gimbal_yaw;
//            INFO(gimbal_yaw);
            if(vec_gimbal_yaw.size() < 120)    // 缓存120组历史陀螺仪数据zz
            {
                vec_gimbal_yaw.push_back(gimbal_yaw);
            }else
            {
                vec_gimbal_yaw.erase(vec_gimbal_yaw.begin());
                vec_gimbal_yaw.push_back(gimbal_yaw);
            }

            last_gimbal_yaw = curr_gimbal_yaw;
        }
        //        if(prdIdx % 100==0)
        //            printf("stm32 fd: %d\tgimbal fd: %d\r\n", serial_.fd, serial_gimbal_.fd);
        END_THREAD;
    }
}

void ImageProduceProcess::GetSTM32()
{
    while(1){
            int8_t modae;
            serial_.read_data(&rx_data, mode_, color_, bullet_speed_, cancel_kalman_);
//                        if(mode_ == 0)
//                            cout << " mode=auto_aim  ";
//                        else
//                            cout << " mode=buff_aim  ";
//                        if(color_ == 0)
//                            cout << "  BLUE  ";
//                        else
//                            cout << "  RED  ";
//            //            cout << "bullet_speed = " << bullet_speed_;
//                        if(cancel_kalman_ == 0)
//                            cout << "  use_kalman "<<endl;
//                        else
//                            cout << "  cancel_kalman "<<endl;
//            INFO(bullet_speed_);
        END_THREAD;
    }
}

// 图像处理线程
void ImageProduceProcess::ImageProcess()
{
    Param_ para_armor, para_buff;
    namedWindow("control");
    createTrackbar("gray_th", "control", &para_armor.gray_th, 255);
    createTrackbar("color_th", "control", &para_armor.color_th, 255);
    cv::createTrackbar("offset_x","control",&para_armor.offset_x,200);
    cv::createTrackbar("offset_y","control",&para_armor.offset_y,200);
    cv::createTrackbar("Qp","control",&km_Qp,1000);
    cv::createTrackbar("Qv","control",&km_Qv,1000);
    cv::createTrackbar("rp","control",&km_Rp,1000);
    cv::createTrackbar("rv","control",&km_Rv,1000);
    cv::createTrackbar("t","control",&km_t,10);
    cv::createTrackbar("pt","control",&km_pt,500);
    createTrackbar("history_index", "control", &history_index, 120);

    Mat image;
    float angle_x = 0.0, angle_y = 0.0, distance = 0.0, theta_y = 0.0;
    float angle_x_i = 0.0, angle_y_i = 0.0, distance_i = 0.0;
    float gimbal_angle_x = 0.0, predict_angle_x = 0.0;
    SolveAngle solve_angle("/home/cz/rm-vision/camera4mm.xml", -20, 42.5, -135, 0);
    //    SolveAngle solve_angle("/home/cz/rm-vision/camera4mm.xml", -0, 0, 0, 0);
    ZeYuPredict zeyu_predict(0.01, 0.01, 0.01, 0.01, 3, 3);

    while(1)
    {

        while(prdIdx - csmIdx == 0);
        data[csmIdx % BUFFER_SIZE].img.copyTo(image);
        ++csmIdx;
        imshow("src", image);
        double t1 = getTickCount();
        bool find_flag = ArmorDetectTask(image, para_armor);   // 装甲板检测
        if(find_flag)
        {
            solve_angle.getAngle(para_armor.points_2d, 20, angle_x, angle_y, distance, theta_y);
            solve_angle.getAngle_ICRA(para_armor.points_2d, 20, angle_x_i, angle_y_i, distance_i);

            putText(image, "origin z :" + to_string(distance) + " x: " + to_string(angle_x) + " y :" + to_string(angle_y) + "theta_y :" + to_string(theta_y)
                    , Point(0,20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
            putText(image, "orICRA z :" + to_string(distance_i) + " x: " + to_string(angle_x_i) + " y :" + to_string(angle_y_i)
                    , Point(0,40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));

            //---predict
            protectDate(km_Qp, km_Qv, km_Rp, km_Rv, km_t, km_pt);

            km_pt = distance/2000*km_pt+30;
            //        float max_limit_angle_y = 10;
            //        float min_limit_angle_y = 5;
            //        if(abs(theta_y)>max_limit_angle_y)
            //        {
            //            km_pt = 0;
            //        }else if(abs(theta_y )< max_limit_angle_y && abs(theta_y) > min_limit_angle_y)
            //        {
            //            km_pt = km_pt * (max_limit_angle_y - abs(theta_y))/max_limit_angle_y;
            //        }
            zeyu_predict.setQRT(km_Qp,km_Qv,km_Rp,km_t,km_pt);
            if(serial_gimbal_.success_)   // gimbal is successed
            {
                //            cout << "true" << endl;
                if(history_index==0)
                    history_index = 1;
                vector<float> vec_gimbal_yaw_tmp = vec_gimbal_yaw;
                if(vec_gimbal_yaw_tmp.size()>history_index)
                    gimbal_angle_x = vec_gimbal_yaw_tmp.at(vec_gimbal_yaw_tmp.size()-history_index);
                else if(vec_gimbal_yaw_tmp.size()==0)
                    gimbal_angle_x = 0.0;
                else
                    gimbal_angle_x = vec_gimbal_yaw_tmp.at(0);
                //                    printf("gimbla_angle%f\r\n", gimbal_angle_x);
                float gim_and_pnp_angle_x = -gimbal_angle_x + angle_x;  // 陀螺仪角度+pnp解析角度，从而获得敌人绝对角度
//                INFO(gim_and_pnp_angle_x);
                predict_angle_x = zeyu_predict.run_position(gim_and_pnp_angle_x);   // kalman滤波预测
                predict_angle_x += gimbal_angle_x;
                angle_x = predict_angle_x;
            }
        }

        //--/predict
        limit_angle(angle_x, 4);
//        tx_data.get_xy_data(-angle_x*100, -angle_y*100, find_flag);
//        serial_.send_data(tx_data);


                imshow("image",image);
//                waitKey(0);
                if(waitKey(1)>0)
                    end_thread_flag = true;
        //        usleep(1);
        double t2 = getTickCount();
        double t = (t2 - t1)*1000/getTickFrequency();
//        INFO(t);
        END_THREAD;

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
