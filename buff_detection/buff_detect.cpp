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
#include "buff_detect.h"

bool BuffDetector::DetectBuff(Mat& img, OtherParam other_param)
{

    GaussianBlur(img, img, Size(3,3),0);
    // **预处理** -图像进行相应颜色的二值化
    points_2d.clear();
    vector<cv::Mat> bgr;
    split(img, bgr);
    Mat result_img;
    if(color_ != 0)
    {
        subtract(bgr[2], bgr[1], result_img);
    }else
    {
        subtract(bgr[0], bgr[2], result_img);
    }
    Mat binary_color_img;
#ifdef TEST_OTSU
    double th = threshold(result_img, binary_color_img, 50, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
    if(th-10>0)
        threshold(result_img, binary_color_img, th-10, 255, CV_THRESH_BINARY);
#endif
#ifndef TEST_OTSU
    threshold(result_img, binary_color_img, color_th_, 255, CV_THRESH_BINARY);
#endif
    //        Mat element = getStructuringElement(MORPH_RECT, Size(5,5));
    //        morphologyEx(binary_color_img,binary_color_img,MORPH_CLOSE,element);
    //        dilate(img, img, element);
#ifdef DEBUG_BUFF_DETECT
    imshow("mask", binary_color_img);
#endif

#ifdef TEST_OTSU
    if(th < 20)
        return 0;
#endif
    // **寻找击打矩形目标** -通过几何关系
    // 寻找识别物体并分类到object
    vector<Object> vec_target;
    vector<Rect> vec_color_rect;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary_color_img,contours,hierarchy,CV_RETR_CCOMP,CHAIN_APPROX_NONE);
    for(size_t i=0; i < contours.size();i++)
    {

        // 用于寻找小轮廓，没有父轮廓的跳过, 以及不满足6点拟合椭圆
        if(hierarchy[i][3]<0 || contours[i].size() < 6 || contours[static_cast<uint>(hierarchy[i][3])].size() < 6)
            continue;

        // 小轮廓面积条件
        double small_rect_area = contourArea(contours[i]);
        double small_rect_length = arcLength(contours[i],true);
        if(small_rect_length < 10)
            continue;
        // 用于超预测时比例扩展时矩形的判断
        Rect rect = boundingRect(contours[static_cast<uint>(hierarchy[i][3])]);
        vec_color_rect.push_back(rect);

        if(small_rect_area < 200)
            continue;
        // 大轮廓面积条件
        double big_rect_area = contourArea(contours[static_cast<uint>(hierarchy[i][3])]);
        double big_rect_length = arcLength(contours[static_cast<uint>(hierarchy[i][3])],true);
        if(big_rect_area < 300)
            continue;
        if(big_rect_length < 50)
            continue;
        // 能量机关扇叶进行拟合
        Object object;
#ifdef FUSION_MINAREA_ELLIPASE

        object.small_rect_=fitEllipse(contours[i]);
        object.big_rect_ = fitEllipse(contours[static_cast<uint>(hierarchy[i][3])]);
#else
        object.small_rect_=minAreaRect(contours[i]);
        object.big_rect_ = minAreaRect(contours[static_cast<uint>(hierarchy[i][3])]);
#endif

#ifdef DEBUG_DRAW_CONTOURS
        Point2f small_point_tmp[4];
        object.small_rect_.points(small_point_tmp);
        Point2f big_point_tmp[4];
        object.big_rect_.points(big_point_tmp);
        for(int k=0;k<4;k++)
        {
            line(img, small_point_tmp[k],small_point_tmp[(k+1)%4], Scalar(0, 255, 255), 1);
            line(img, big_point_tmp[k],big_point_tmp[(k+1)%4], Scalar(0, 0, 255), 1);
        }

#endif
#ifdef FUSION_MINAREA_ELLIPASE
        object.diff_angle=fabsf(object.big_rect_.angle-object.small_rect_.angle);

        if(object.small_rect_.size.height/object.small_rect_.size.width < 3)
        {
            if(object.diff_angle<100 && object.diff_angle>80)
            {
#endif
#ifndef  FUSION_MINAREA_ELLIPASE
                float small_rect_size_ratio;
                if(object.small_rect_.size.width > object.small_rect_.size.height)
                {
                    small_rect_size_ratio = object.small_rect_.size.width/object.small_rect_.size.height;
                }else {
                    small_rect_size_ratio = object.small_rect_.size.height/object.small_rect_.size.width;
                }
#endif

#ifdef FUSION_MINAREA_ELLIPASE
                float small_rect_size_ratio;
                small_rect_size_ratio = object.small_rect_.size.height/object.small_rect_.size.width;
#endif
                // 根据轮廓面积进行判断扇叶类型

                double area_ratio = area_ratio_/100;
                if(small_rect_area * 12 >big_rect_area && small_rect_area* area_ratio<big_rect_area
                        && small_rect_size_ratio > 1 && small_rect_size_ratio < 3.0f)
                {
                    object.type_ = ACTION;  // 已经激活类型
                }else if(small_rect_area * area_ratio>=big_rect_area && small_rect_area *2 < big_rect_area
                         && small_rect_size_ratio > 1 && small_rect_size_ratio < 3.0f)
                {

                    // 更新世界坐标系顺序
                    object.type_ = INACTION;    // 未激活类型
                    object.UpdateOrder();
                    object.KnowYourself(binary_color_img);
                    vec_target.push_back(object);
                }else
                {
                    object.type_ = UNKOWN;    // 未激活类型
                }
#ifdef AREA_LENGTH_ANGLE
                switch (AREA_LENGTH_ANGLE)
                {
                case 1:
                {
                    double multiple_area=fabs(big_rect_area/small_rect_area);
                    putText(img, to_string(multiple_area), Point2f(50,50)+ object.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));
                }break;
                case 2:
                {
                    double multiple_length=fabs(big_rect_length/small_rect_length);
                    putText(img, to_string(multiple_length), Point2f(50,50)+ object.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));
                }break;
                case 3:
                {
                    putText(img, to_string(object.diff_angle), Point2f(-20,-20)+ object.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));
                }break;
                }
#endif

#ifdef FUSION_MINAREA_ELLIPASE
            }
        }
#endif
    }
    // 遍历所有结果并处理\选择需要击打的目标
    Object final_target;
    bool find_flag = false;
    float diff_angle = 1e8;
    // 你需要击打的能量机关类型 1(true)击打未激活 0(false)击打激活
    for(size_t i=0; i < vec_target.size(); i++)
    {
        Object object_tmp = vec_target.at(i);
        // 普通模式击打未激活机关
        if(object_tmp.type_ == INACTION){
            find_flag = true;
            float ang = fabs(vec_target[i].diff_angle-90.0f);
            if(ang < diff_angle)
            {
                final_target = vec_target.at(i);
                diff_angle = ang;
            }
            putText(img, "final_target", Point2f(10,-50)+ final_target.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));

            Point2f buff_offset = Point2f(buff_offset_x_ - 100, 100 - buff_offset_y_);
            vector<Point2f> vec_points_2d_tmp;
            for(size_t k=0; k < 4; k++)
            {
                vec_points_2d_tmp.push_back(final_target.points_2d_.at(k) + buff_offset);
            }
            points_2d = vec_points_2d_tmp;
            buff_angle_ = final_target.angle_;
#ifdef DEBUG_PUT_TEST_ANGLE
            for(size_t j = 0; j < 4; j++)
            {
                putText(img, to_string(j), Point2f(5,5)+ final_target.points_2d_[j], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));
            }
#endif
        }
    }
    if(find_flag){


#ifdef DEBUG_PUT_TEST_TARGET
//        putText(img, "<<---attack here"/*to_string(object_tmp.angle_)*/, Point2f(5,5)+ final_target.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
#endif
#ifdef DEBUG_DRAW_TARGET
        final_target.DrawTarget(img);
#endif
    }
    return find_flag;
}




int BuffDetector::BuffDetectTask(Mat& img, OtherParam other_param)
{
    color_ = other_param.color;
//    gimbal=other_param.gimbal_data + (pitch_offset-2000)/100;
    buff_offset_x_ = begin_offset_x_ + 3*other_param.buff_offset_x;
    buff_offset_y_ = begin_offset_y_ + 3*other_param.buff_offset_y;
    bool find_flag = DetectBuff(img,other_param);
    int command = 0;
    bool is_change = false;
    if(find_flag)
    {
        find_cnt ++;
        if(find_cnt % 15==0)
        {
            //            direction_tmp = getDirection(buff_angle_);
            direction_tmp = getSimpleDirection(buff_angle_);
        }
        Point2f world_offset;
        #define DIRECTION_FILTER
#ifdef DIRECTION_FILTER
        float world_offset_x = world_offset_x_ - 500;
        float world_offset_y = 800 - pow((640000 - pow(world_offset_x, 2)), 0.5);
        float pre_angle;
        if(direction_tmp == 1)  // shun
        {
            world_offset = Point2f(-world_offset_x, -world_offset_y);
            pre_angle = atan(world_offset_x/(800-world_offset_y));
        }
        else if(direction_tmp == -1)// ni
        {
            world_offset = Point2f(world_offset_x, -world_offset_y);
            pre_angle = -atan(world_offset_x/(800-world_offset_y));
        }
        else
        {
            world_offset = Point2f(0, 0);
            pre_angle = 0;
        }
        //        cout << "direction " << direction_tmp << endl;

#else
        world_offset = Point2f(world_offset_x_ - 500, world_offset_y_  - 500);
#endif
        solve_angle_long_.Generate3DPoints(2, world_offset);
        solve_angle_long_.getBuffAngle(1,points_2d, BULLET_SPEED
                                       , buff_angle_, pre_angle, gimbal
                                       , angle_x_, angle_y_, distance_);

#ifdef LINER_OFFSET_PITCH
        angle_y_ = angle_y_ + solve_angle_long_.buff_h * (pitch_offset-2000)/160000;
#endif


    }
    //    attack.run(find_flag,angle_x_,angle_y_,target_size,gimbal,direction_tmp);
    command = auto_control.run(angle_x_, angle_y_, find_flag, diff_angle_);

    //    pro_predict_mode_flag = auto_control.GetProPredictFlag();

#ifdef DEBUG_PLOT //0紫 1橙
//    w_->addPoint((auto_control.fire_cnt%2==0)*100+100, 0);
//    w_->addPoint(solve_angle_long_.buff_h, 0);

    w_->plot();
#endif
    return command;
}

int BuffDetector::getSimpleDirection(float angle)
{
    diff_angle_ = angle - last_angle_;
    last_angle_ = angle;
    if(fabs(diff_angle_) < 10 && fabs(diff_angle_) > 1e-6)
    {
        d_angle_ = (1 - r) * d_angle_ + r * diff_angle_;
    }
    if(d_angle_ > 2)
        return 1;
    else if(d_angle_ < -2)
        return -1;
    else
        return 0;
}


void Object::UpdateOrder()
{
    points_2d_.clear();
#ifdef FUSION_MINAREA_ELLIPASE
    Point2f points[4];
    small_rect_.points(points);
    Point2f point_up_center = (points[0] + points[1])/2;
    Point2f point_down_center = (points[2] + points[3])/2;
    double up_distance = Point_distance(point_up_center, big_rect_.center);
    double down_distance = Point_distance(point_down_center, big_rect_.center);
    if(up_distance > down_distance)
    {
        angle_ = small_rect_.angle;
        points_2d_.push_back(points[0]);points_2d_.push_back(points[1]);
        points_2d_.push_back(points[2]);points_2d_.push_back(points[3]);
    }else
    {
        angle_ = small_rect_.angle + 180;
        points_2d_.push_back(points[2]);points_2d_.push_back(points[3]);
        points_2d_.push_back(points[0]);points_2d_.push_back(points[1]);
    }
#else
    float width = small_rect_.size.width;
    float height = small_rect_.size.height;
    Point2f points[4];
    small_rect_.points(points);
    if(width >= height)
    {
        Point2f point_up_center = (points[0] + points[3])/2;
        Point2f point_down_center = (points[1] + points[2])/2;
        float up_distance = Point_distance(point_up_center, big_rect_.center);
        float down_distance = Point_distance(point_down_center, big_rect_.center);
        if(up_distance <= down_distance)
        {
            angle_ = 90 - small_rect_.angle;
            points_2d_.push_back(points[1]);points_2d_.push_back(points[2]);
            points_2d_.push_back(points[3]);points_2d_.push_back(points[0]);

        }else
        {
            angle_ = 270 - small_rect_.angle;
            points_2d_.push_back(points[3]);points_2d_.push_back(points[0]);
            points_2d_.push_back(points[1]);points_2d_.push_back(points[2]);
        }
    }else
    {
        Point2f point_up_center = (points[0] + points[1])/2;
        Point2f point_down_center = (points[2] + points[3])/2;
        float up_distance = Point_distance(point_up_center, big_rect_.center);
        float down_distance = Point_distance(point_down_center, big_rect_.center);
        if(up_distance <= down_distance)
        {
            angle_ = - small_rect_.angle;
            points_2d_.push_back(points[2]);points_2d_.push_back(points[3]);
            points_2d_.push_back(points[0]);points_2d_.push_back(points[1]);

        }else
        {
            angle_ = 180 - small_rect_.angle;
            points_2d_.push_back(points[0]);points_2d_.push_back(points[1]);
            points_2d_.push_back(points[2]);points_2d_.push_back(points[3]);
        }
    }
#endif
}


int GetRectIntensity(const Mat &img, Rect rect){
    if(rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y < 1
            || rect.width + rect.x > img.cols || rect.height + rect.y > img.rows)
        return 255;
    Mat roi = img(Range(rect.y, rect.y + rect.height), Range(rect.x, rect.x + rect.width) );
    //        imshow("roi ", roi);
    int average_intensity = static_cast<int>(mean(roi).val[0]);
    return average_intensity;
}

void Object::KnowYourself(Mat &img)
{
    Point2f vector_height = points_2d_.at(0) - points_2d_.at(3);
    //    vector_height = Point2f(vector_height.x * 0.5 , vector_height.y * 0.5);
    Point left_center = points_2d_.at(3) - vector_height;
    Point right_center = points_2d_.at(2) - vector_height;
    //        circle(img, left_center, 3, Scalar(255), -1);
    //        circle(img, right_center, 3, Scalar(255), 1);

    int width = 5;
    int height = 5;

    Point left1 = Point(left_center.x - width, left_center.y - height);
    Point left2 = Point(left_center.x + width, left_center.y + height);

    Point right1 = Point(right_center.x - width, right_center.y - height);
    Point right2 = Point(right_center.x + width, right_center.y + height);

    Rect left_rect(left1, left2);
    Rect right_rect(right1, right2);

    //    rectangle(img, left_rect, Scalar(255), 1);
    //    rectangle(img, right_rect, Scalar(255), 1);

    int left_intensity = GetRectIntensity(img, left_rect);
    int right_intensity = GetRectIntensity(img, right_rect);
    if(left_intensity > 10 && right_intensity > 10)
    {
        type_ = ACTION;
    }else{
        type_ = INACTION;
    }
    putText(img, to_string(left_intensity), left_center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255));
    putText(img, to_string(right_intensity), right_center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255));
#ifdef IMSHOW_2_ROI
    imshow("test", img);
#endif
}



double Point_distance(Point2f p1,Point2f p2)
{
    double Dis=pow(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2),0.5);
    return Dis;
}
