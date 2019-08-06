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
#include "armor_detect.h"

// 计算两个点之间的距离
double calc_distance(Point2f p1, Point2f p2)
{
    return pow(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2),0.5);
}

armor::armor(){}

armor::armor(const LED_Stick& L1, const LED_Stick& L2){
    Led_stick[0]= L1;
    Led_stick[1]= L2;
    error_angle = fabs(L1.rect.angle - L2.rect.angle);

    rect.width = abs(static_cast<int>(L1.rect.center.x - L2.rect.center.x));
    rect.height = static_cast<int>((L1.rect.size.height + L1.rect.size.height)/2);
    center.x = static_cast<int>((L1.rect.center.x + L2.rect.center.x)/2);
    center.y = static_cast<int>((L1.rect.center.y + L2.rect.center.y)/2);
    rect.x = center.x - rect.width/3;
    rect.y = center.y - rect.height/3;
    rect.width*= 2.0/3;
    rect.height*= 2.0/3;
}

void armor::draw_rect( Mat& img, Point2f roi_offset_point) const
{
    rectangle(img, rect + Point_<int>(roi_offset_point), Scalar(255,255,255), 1);
}

void armor::draw_spot(Mat &img, Point2f roi_offset_point) const
{
    circle(img, center + Point_<int>(roi_offset_point), int(rect.height/4), Scalar(0,0,255), -1);
}


int armor::get_average_intensity(const Mat& img) {
    if(rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y < 1
            || rect.width + rect.x > img.cols || rect.height + rect.y > img.rows)
        return 255;
    Mat roi = img(Range(rect.y, rect.y + rect.height), Range(rect.x, rect.x + rect.width) );
    //        imshow("roi ", roi);
    average_intensity = static_cast<int>(mean(roi).val[0]);
    return average_intensity;
}


void armor::max_match(vector<LED_Stick>& LED,size_t i,size_t j){
    RotatedRect R, L;
    if(Led_stick[0].rect.center.x > Led_stick[1].rect.center.x)
    {
        R = Led_stick[0].rect;
        L = Led_stick[1].rect;
    }else
    {
        R = Led_stick[1].rect;
        L = Led_stick[0].rect;
    }

    float angle_8 = L.angle - R.angle;
    //    cout << L.angle << " "<< R.angle << endl;
    if(angle_8 < 1e-3f)
        angle_8 = 0.0f;
    float f = error_angle + 0.5 * angle_8;
    if(!LED.at(i).matched && !LED.at(j).matched )
    {

        LED.at(i).matched = true;
        LED.at(i).match_index = j;
        LED.at(j).matched = true;
        LED.at(j).match_index = i;
        LED.at(i).match_factor = f;
        LED.at(j).match_factor = f;
    }
    if(LED.at(i).matched && !LED.at(j).matched)
    {
        if(f < LED.at(i).match_factor)
        {
            LED.at(LED.at(i).match_index).matched = false;
            LED.at(i).match_factor = f;
            LED.at(i).match_index = j;
            LED.at(j).matched = true;
            LED.at(j).match_factor = f;
            LED.at(j).match_index = i;

        }
    }
    if(LED.at(j).matched && !LED.at(i).matched)
    {
        if(f < LED.at(j).match_factor )
        {
            LED.at(LED.at(j).match_index).matched = false;
            LED.at(j).match_factor = f;
            LED.at(j).match_index = i;
            LED.at(i).matched = true;
            LED.at(i).match_factor = f;
            LED.at(i).match_index = j;
        }
    }
    if(LED.at(j).matched && LED.at(i).matched
            && LED.at(i).match_factor > f && LED.at(j).match_factor > f)
    {
        LED.at(LED.at(j).match_index).matched = false;
        LED.at(LED.at(i).match_index).matched = false;
        LED.at(i).matched = true;
        LED.at(i).match_factor = f;
        LED.at(i).match_index = j;
        LED.at(j).matched = true;
        LED.at(j).match_factor = f;
        LED.at(j).match_index = i;
    }
}

bool armor::is_suitable_size(void) const
{
    // 两个灯条体型相似
    if(Led_stick[0].rect.size.height*0.7f < Led_stick[1].rect.size.height
            && Led_stick[0].rect.size.height*1.3f > Led_stick[1].rect.size.height)
    {
        float armor_width = fabs(Led_stick[0].rect.center.x - Led_stick[1].rect.center.x);
        if(armor_width > Led_stick[0].rect.size.width
                && armor_width > Led_stick[1].rect.size.width
                && armor_width > (Led_stick[0].rect.size.width+Led_stick[1].rect.size.width)*3)
        {
            float h_max = (Led_stick[0].rect.size.height + Led_stick[1].rect.size.height)/2.0f;
            // 两个灯条高度差不大
            if(fabs(Led_stick[0].rect.center.y - Led_stick[1].rect.center.y) < 0.8f* h_max )
            {
                // 长宽比判断
                if(h_max*4.0f > rect.width && h_max < 1.2f* rect.width)
                {
                    return true;
                }
            }
        }
    }
    return false;
}


Rect ArmorDetector::GetRoi(const Mat &img)
{
    Size img_size = img.size();
    Rect rect_tmp = last_target_;
    Rect rect_roi;
    if(rect_tmp.x == 0 || rect_tmp.y == 0
            || rect_tmp.width == 0 || rect_tmp.height == 0
            || lost_cnt_ >= 15 || detect_cnt_%100 == 0
        #ifndef FORCE_CHANGE_CAMERA
            || update_cap_cnt < 20
        #endif
            )
    {
        last_target_ = Rect(0,0,img_size.width, img_size.height);
        rect_roi = Rect(0,0,img_size.width, img_size.height);
        return rect_roi;
    }
    else
    {
        float scale = 2;
        if (lost_cnt_ < 30)
            scale = 3;
        else if(lost_cnt_ <= 60)
            scale = 4;
        else if(lost_cnt_ <= 120)
            scale = 5;

        int w = int(rect_tmp.width * scale);
        int h = int(rect_tmp.height * scale);
        int x = int(rect_tmp.x - (w - rect_tmp.width)*0.5f);
        int y = int(rect_tmp.y - (h - rect_tmp.height)*0.5f);

        rect_roi = Rect(x, y, w, h);

        if(makeRectSafe(rect_roi, img_size)== false)
        {
            rect_roi = Rect(0,0,img_size.width, img_size.height);
        }
    }
    return rect_roi;
}

bool ArmorDetector::DetectArmor(Mat &img, Rect roi_rect)
{
    // **预处理** -图像进行相应颜色的二值化
    Mat roi_image = img(roi_rect);
    Point2f offset_roi_point(roi_rect.x, roi_rect.y);
    vector<LED_Stick> LED_Stick_v;  // 声明所有可能的灯条容器
    Mat binary_brightness_img, binary_color_img, gray;
    cvtColor(roi_image,gray,COLOR_BGR2GRAY);
    //    Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
    //    dilate(img, img, element);
    vector<cv::Mat> bgr;
    split(roi_image, bgr);
    Mat result_img;
    if(color_ == 0)
    {
        subtract(bgr[2], bgr[1], result_img);
    }else
    {
        subtract(bgr[0], bgr[2], result_img);
    }

    threshold(gray, binary_brightness_img, gray_th_, 255, CV_THRESH_BINARY);
    threshold(result_img, binary_color_img, color_th_, 255, CV_THRESH_BINARY);

    // **提取可能的灯条** -利用灯条（灰度）周围有相应颜色的光圈包围
    //    printf("bin_th = %d, color_th = %d\r\n", show_bin_th, show_color_th);
#ifdef SHOW_BINARY_IMAGE
    imshow("binary_brightness_img", binary_brightness_img);
    imshow("binary_color_img", binary_color_img);
#endif
    vector<vector<Point>> contours_light;
    vector<vector<Point>> contours_brightness;
    findContours(binary_color_img, contours_light, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    findContours(binary_brightness_img, contours_brightness, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    //#pragma omp for
    for(size_t i = 0; i < contours_brightness.size(); i++)
    {
        double area = contourArea(contours_brightness[i]);
        if (area < 20.0 || 1e5 < area) continue;
        for(size_t ii = 0; ii < contours_light.size(); ii++)
        {
            if(pointPolygonTest(contours_light[ii], contours_brightness[i][0], false) >= 0.0 )
            {
                double length = arcLength(contours_brightness[i], true); // 灯条周长
                if (length > 15 && length <4000)
                {                    // 使用拟合椭圆的方法要比拟合最小矩形提取出来的角度更精确
                    RotatedRect RRect = fitEllipse(contours_brightness[i]);
#ifdef SHOW_LIGHT_CONTOURS
                    // 旋转矩形提取四个点
                    Point2f rect_point[4];
                    RRect.points(rect_point);
                    for (int i = 0; i < 4 ; i++)
                    {
                        line(img, rect_point[i]+offset_roi_point, rect_point[(i+1)%4]+offset_roi_point, Scalar(255,0,255),1);
                    }
#endif
                    // 角度换算，将拟合椭圆0~360 -> -180~180
                    if(RRect.angle>90.0f)
                        RRect.angle =  RRect.angle - 180.0f;
#ifdef SHOW_LIGHT_PUT_TEXT
                    putText(img, to_string(RRect.angle), RRect.center + Point2f(2,2) + offset_roi_point, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
#endif
                    if (fabs(RRect.angle) <= 30)  // 超过一定角度的灯条不要
                    {
                        LED_Stick r(RRect);
                        LED_Stick_v.push_back(r);
                    }
                }
                break;
            }
        }
    }

    // **寻找可能的装甲板** -遍历每个可能的灯条, 两两灯条拟合成装甲板进行逻辑判断
    for(size_t i = 0; i < LED_Stick_v.size() ; i++)
    {
        for(size_t j = i + 1; j < LED_Stick_v.size() ; j++)
        {
            armor arm_tmp( LED_Stick_v.at(i), LED_Stick_v.at(j) );
            if (arm_tmp.error_angle < 8.0f)
            {
#ifdef SHOW_ARMOR_PUT_TEXT
                putText(img, to_string(arm_tmp.rect.width/(arm_tmp.rect.height+0.0001)), arm_tmp.center + Point_<int>(offset_roi_point) , FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
#endif
                // TODO(cz): 推荐加入灯条宽度要小于装甲板宽度的条件
                if(arm_tmp.is_suitable_size())
                {
                    // TODO(cz): 推荐使用255值的面积进行判断
                    if(arm_tmp.get_average_intensity(gray)< 50 )
                    {
                        arm_tmp.max_match(LED_Stick_v, i, j);
                    }
                }
            }
        }
    }

    // **分类装甲板** -根据灯条匹配状态得到最终装甲板
    vector<armor> final_armor_list;
    for(size_t i = 0; i < LED_Stick_v.size() ; i++)
    {
        if(LED_Stick_v.at(i).matched)
        {
            LED_Stick_v.at(LED_Stick_v.at(i).match_index).matched = false; //clear another matching flag
            armor arm_tmp( LED_Stick_v.at(i), LED_Stick_v.at(LED_Stick_v.at(i).match_index));
            final_armor_list.push_back(arm_tmp);
        }
    }

    // **选择装甲板** -根据距离图像中心最短选择
    float dist=1e8;
    bool found_flag = false;
    armor target;
    Point2f roi_center(roi_rect.width/2, roi_rect.height/2);
    float dx,dy;
    for (size_t i = 0; i < final_armor_list.size() ; i++ )
    {
#ifdef FAST_DISTANCE
        dx = fabs(final_armor_list.at(i).center.x - roi_center.x);
        dy = fabs(final_armor_list.at(i).center.y - roi_center.y);
#else
        dx = pow((final_armor_list.at(i).center.x - roi_center.x), 2.0f);
        dy = pow((final_armor_list.at(i).center.y - roi_center.y), 2.0f);
#endif
        if( dx + dy < dist){
            target = final_armor_list.at(i);
            dist = dx + dy;
        }
#ifdef SHOW_DRAW_RECT
        final_armor_list.at(i).draw_rect(img, offset_roi_point);
#endif
        found_flag = true;
    }
#ifdef SHOW_ROI_RECTANGLE
    rectangle(img, roi_rect,Scalar(255, 0, 255),1);
#endif
    // **计算装甲板四个点顶点** -用于pnp姿态结算
    // TODO(cz): 四个点的不同的bug修复
    RotatedRect target_rect;
    if(found_flag)
    {
#ifdef SHOW_DRAW_SPOT
        target.draw_spot(img, offset_roi_point);
#endif
        Point2f point_tmp[4];
        Point2f point_2d[4];
        // 左右灯条分类，本别提取装甲板四个外角点
        RotatedRect R, L;
        if(target.Led_stick[0].rect.center.x > target.Led_stick[1].rect.center.x)
        {
            R = target.Led_stick[0].rect;
            L = target.Led_stick[1].rect;
        }else
        {
            R = target.Led_stick[1].rect;
            L = target.Led_stick[0].rect;
        }
        L.points(point_tmp);
        point_2d[0] = point_tmp[1];
        point_2d[3] = point_tmp[0];
        R.points(point_tmp);
        point_2d[1] = point_tmp[2];
        point_2d[2] = point_tmp[3];
        // 计算补偿，用于调试调整准心
        Point2f offset_point;
        if(cap_mode_ == 0)
        {
            offset_point = Point2f(100, 100) - Point2f(short_offset_x_,short_offset_y_);
        }
        else
        {
            offset_point = Point2f(100, 100) - Point2f(long_offset_x_,long_offset_y_);
        }

        points_2d_.clear();
        vector<Point2f> points_roi_tmp;
        for(int i=0;i<4;i++)
        {
            points_roi_tmp.push_back(point_2d[i] + offset_roi_point);
            points_2d_.push_back(point_2d[i] + offset_roi_point +offset_point);
//            putText(img, to_string(i), points_2d_.at(i), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
//            circle(img, points_2d_.at(i), 5, Scalar(255, 255, 255), -1);
//            circle(img, points_2d_.at(i), 3, Scalar(i*50, i*50, 255), -1);
        }
        // 计算当前装甲板类型，到后面task中还有滤波，可以有误差
        float armor_h = target.rect.height;
        float armor_w = target.rect.width;
        if(armor_w / armor_h < 3.3f)
            is_small_ = 1;
        else
            is_small_ = 0;

        //计算ROI的相关参数
        last_target_ = boundingRect(points_roi_tmp);
        rectangle(img, last_target_,Scalar(255,255,255), 1);
        lost_cnt_ = 0;
    }else {
        //计算ROI的相关参数
        lost_cnt_ ++;
    }
    detect_cnt_++;
    return found_flag;
}


int ArmorDetector::ArmorDetectTask(Mat &img,OtherParam other_param)
{
    //    double t1 = getTickCount();
    // 取外部参数的值
    color_ = other_param.color;
    cap_mode_ = other_param.cap_mode;

    // 获取历史时刻roi使能
#ifdef ROI_ENABLE
    Rect roi = GetRoi(img);
#else
    Size img_size = img.size();
    Rect roi = Rect(0,0, img_size.width, img_size.height);
#endif
    if(DetectArmor(img, roi))
    {
        // 精简角度解算算法参数
#ifdef SIMPLE_SOLVE_ANGLE_FOR_ARMOR_DETECT
        SimpleSolveAngle short_simple_solve(511.8909f, 511.8642f, 334.1404f, 172.4067f, 700);
        SimpleSolveAngle long_simple_solve(1300.3841f, 1300.2605f, 331.3789f, 269.5436f, 1500);
        Point2f screen_point;
        for(int i = 0; i <4; i++)
        {
            screen_point += points_2d_.at(i);
        }
        screen_point /= 4;
        float dh = points_2d_.at(2).y - points_2d_.at(1).y;
#endif
//        DrawTarget(img);
        bool final_armor_type = getTypeResult(is_small_);
        //                    float angle_x, angle_y, distance;
        if(cap_mode_ == 0) // close
        {
#ifdef SIMPLE_SOLVE_ANGLE_FOR_ARMOR_DETECT
            short_simple_solve.getAngle(screen_point.x, screen_point.y, dh, angle_x_, angle_y_, distance_);

#else
            solve_angle_.Generate3DPoints((uint)final_armor_type, Point2f());
            solve_angle_.getAngle(points_2d_, 15,angle_x_,angle_y_,distance_);   // pnp姿态结算
#endif
        }
        else                    // far
        {
#ifdef SIMPLE_SOLVE_ANGLE_FOR_ARMOR_DETECT
            long_simple_solve.getAngle(screen_point.x, screen_point.y, dh, angle_x_, angle_y_, distance_);
#else
            solve_angle_long_.Generate3DPoints((uint)final_armor_type, Point2f());
            solve_angle_long_.getAngle(points_2d_, 15,angle_x_, angle_y_ ,distance_);   // pnp姿态结算
#endif
        }

#ifdef DEBUG_PLOT //0紫 1橙
        w_->addPoint(distance_, 0);
//        w_->addPoint(angle_y_, 1);
        w_->plot();
#endif
#ifdef PREDICT
        // 间隔时间计算
        double t_tmp = getTickCount();
        double delta_t = (t_tmp - t_start_)*1000/getTickFrequency();
        t_start_ = t_tmp;

        protectDate(km_Qp_, km_Qv_, km_Rp_, km_Rv_, km_t_, km_pt_);
        //        float pre_time = distance_/10000*static_cast<float>(km_pt_)+10.0f;
        zeyu_predict_.setQRT(km_Qp_,km_Qv_,km_Rp_,km_t_,km_pt_);
        float gim_and_pnp_angle_x = -other_param.gimbal_data + angle_x_;
        // 速度计算
        float angle_v = (gim_and_pnp_angle_x - last_angle)/delta_t;
        last_angle = gim_and_pnp_angle_x;
        // 加速度计算
#if(0)
        float u = (last_v - last_last_v)/delta_t;
        last_last_v = last_v;
        last_v = angle_v;
#else
        float u = (angle_v - last_v)/delta_t;
        last_v = angle_v;
#endif

        float predict_angle_x = zeyu_predict_.run_position(gim_and_pnp_angle_x);   // kalman滤波预测
//        float predict_angle_x = zeyu_predict_.run_position(gim_and_pnp_angle_x, angle_v);   // kalman加入速度滤波预测
//        float predict_angle_x = zeyu_predict_.run_position(gim_and_pnp_angle_x, angle_v, u);   // kalman加入加速度滤波预测
        // ------ 二次拟合数据 ------
        //        double t_tmp = getTickCount();
        //        double detla_t = (t_tmp - t_start_) * 1000 / getTickFrequency();
        //        predict_.setRecord(predict_angle_x, detla_t);
        //        predict_angle_x = predict_.predict(detla_t + 10);
        //        predict_angle_x += other_param.gimbal_data;
        // ------ 二次拟合数据 ------
        angle_x_ = predict_angle_x;
#endif
        if(distance_ > 5000)
            return 0;
        return 1;

    }else
    {
        angle_x_ = 0;
        angle_y_ = 0;
        distance_ = 0;
        dist_ = 0;
        return 0;
    }
}

bool ArmorDetector::getTypeResult(bool is_small)
{
    if (history_.size() < filter_size_){
        history_.push_back(is_small);
    }
    else {
        history_.push_back(is_small);
        history_.pop_front();
    }

    int vote_cnt[2] = {0};
    for (std::list<bool>::const_iterator it = history_.begin(); it != history_.end(); ++it){
        *it == 0 ? ++vote_cnt[0] : ++vote_cnt[1];
    }

    if (vote_cnt[0] == vote_cnt[1])
        return is_small;
    return vote_cnt[0] > vote_cnt[1] ? 0 : 1;
}

bool ArmorDetector::chooseCamera(int short_distance, int long_distance, bool last_mode)
{
    // 没找到目标距离为0
    //找不到目标时自动切换摄像头寻找目标
    bool input_type = last_mode;
    bool temp_type = last_mode;
    bool output_type = last_mode;
    if(distance_ == 0)
    {
        if(lost_cnt_ % 200 == 0 && lost_cnt_ != 0)
        {
            if(last_mode == true)
                temp_type =  false;
            else
                temp_type =  true;
        }else
        {
            temp_type = last_mode;
        }
    }else
    {
        if(dist_ == 0)
            dist_ = distance_;
        else
            dist_ = (1-r_)*dist_ + r_*distance_;
        if(dist_ > long_distance && last_mode == 0)
            temp_type = true;
        else if(dist_ < short_distance && last_mode == 1)
            temp_type = false;
        else
            temp_type = last_mode;
    }
    // 强制条件限制了切换帧数10帧
    update_cap_cnt++;
    if(input_type == temp_type)
    {
        output_type =  temp_type;
    }
    else
    {
        if(update_cap_cnt > 300)
        {
            update_cap_cnt = 0;
            output_type = temp_type;
        }else
        {
            output_type = input_type;
        }
    }
    return output_type;
}
