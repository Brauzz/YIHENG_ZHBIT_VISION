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

bool BuffDetector::DetectBuff(Mat& img)
{
    // **预处理** -图像进行相应颜色的二值化
    GaussianBlur(img, img, Size(3,3),0);
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
    double th = threshold(result_img, binary_color_img, 50, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
    if(th-25>0)
        threshold(result_img, binary_color_img, th-25, 255, CV_THRESH_BINARY);
    //        Mat element = getStructuringElement(MORPH_RECT, Size(5,5));
    //        morphologyEx(binary_color_img,binary_color_img,MORPH_CLOSE,element);
    //        dilate(img, img, element);
#ifdef DEBUG_BUFF_DETECT
    imshow("mask", binary_color_img);
#endif
    if(th < 20)
        return 0;
    // **寻找击打矩形目标** -通过几何关系
    // 寻找识别物体并分类到object
    vector<Object> vec_target;
    vector<Rect> vec_color_rect;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary_color_img,contours,hierarchy,CV_RETR_CCOMP,CHAIN_APPROX_SIMPLE);
    for(size_t i=0; i < contours.size();i++)
    {
        // 用于超预测时比例扩展时矩形的判断
        if(hierarchy[i][3]<0){
            Rect rect = boundingRect(contours[i]);
            vec_color_rect.push_back(rect);
        }
        // 用于寻找小轮廓，没有父轮廓的跳过, 以及不满足6点拟合椭圆
        if(hierarchy[i][3]<0 || contours[i].size() < 6)
            continue;
        // 小轮廓面积条件
        double small_rect_area = contourArea(contours[i]);
        if(small_rect_area < 50)
            continue;
        // 大轮廓面积条件
        double big_rect_area = contourArea(contours[hierarchy[i][3]]);
        if(big_rect_area < 80)
            continue;
        // 能量机关扇叶进行拟合
        Object object;
        object.small_rect_ = fitEllipse(contours[i]);
        object.big_rect_ = fitEllipse(contours[hierarchy[i][3]]);
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
        if(object.small_rect_.size.height/object.small_rect_.size.width < 3)
        {
            // 根据轮廓面积进行判断扇叶类型
            if(small_rect_area * 30 >big_rect_area && small_rect_area*10<big_rect_area)
            {
                object.type_ = ACTION;  // 已经激活类型
            }else if(small_rect_area * 12>big_rect_area && small_rect_area *4 < big_rect_area)
            {
                object.type_ = INACTION;    // 未激活类型
            }
            // 更新世界坐标系顺序
            object.UpdateOrder();
            // 根据距离计算超预测点
            object.UpdataPredictPoint();
            circle(img , object.test_point_, 3, Scalar(22,255,25));
            vec_target.push_back(object);
        }
    }
    // 遍历所有结果并处理\选择需要击打的目标
    //     TODO(cz): 超预测写了一版基础，未加入识别到5个激活目标后再进入超预测的逻辑，仅供参考
    Object final_target;
    bool find_flag = false;
    bool do_you_find_inaction = true;  // 你需要击打的能量机关类型 1(true)击打未激活 0(false)击打激活
    for(int i=0; i < vec_target.size(); i++)
    {
        Object object_tmp = vec_target.at(i);
        if(do_you_find_inaction)
        {
            // 普通模式击打未激活机关
            if(object_tmp.type_ == INACTION)
            {
#ifdef DEBUG_PUT_TEST_TARGET
                putText(img, /*"<<---attack here"*/to_string(object_tmp.angle_), Point2f(5,5)+ object_tmp.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
#endif
                final_target = object_tmp;
                points_2d = final_target.points_2d_;
                buff_angle_ = object_tmp.angle_;
#ifdef DEBUG_PUT_TEST_ANGLE
                for(int j = 0; j < 4; j++)
                {
                    putText(img, to_string(j), Point2f(5,5)+ final_target.points_2d_[j], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));
                }
#endif
                find_flag = true;
                break;
            }
        }else
        {
            // 超预测模式击打目标选择
            bool is_contain = false;
            for(int j=0; j < vec_color_rect.size(); j++)
            {
                //                Object other_object_tmp = vec_target.at(j);
                //                Rect bound_rect_tmp = other_object_tmp.big_rect_.boundingRect();
                Rect bound_rect_tmp = vec_color_rect.at(j);
                rectangle(img, bound_rect_tmp, Scalar(255,0,255));
                if(bound_rect_tmp.contains(object_tmp.test_point_)==true)
                {
                    is_contain = true;
                    break;

                }
            }
            if(is_contain == false)
            {
#ifdef DEBUG_PUT_TEST_TARGET
                putText(img, "<<---attack here", Point2f(5,5)+ object_tmp.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
#endif
                final_target = object_tmp;
                find_flag = true;
                break;
            }
        }

        //        putText(img, to_string(object_tmp.angle_), Point2f(5,5)+ object_tmp.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
    }
#ifdef DEBUG_DRAW_TARGET
    if(find_flag)
        final_target.DrawTarget(img);
#endif
    return find_flag;
}

int BuffDetector::BuffDetectTask(Mat& img, OtherParam other_param)
{
    color_ = other_param.color;
    bool find_flag = DetectBuff(img);
    if(find_flag)
    {
        bool direction_tmp = getDirection(buff_angle_);
        Point2f world_offset;
        //#define DIRECTION_FILTER
#ifdef DIRECTION_FILTER
        if(direction_tmp == 1)  // shun
            world_offset = Point2f(param_.world_offset_x  - 500, param_.world_offset_y - 500);
        else // ni
            world_offset = Point2f(-(param_.world_offset_x  - 500), -(param_.world_offset_y - 500));
        cout << "direction " << direction_tmp << endl;
#else
        world_offset = Point2f(world_offset_x_ - 500, world_offset_y_  - 500);
#endif

        solve_angle_long_.Generate3DPoints(2, world_offset);
        float distance;
        solve_angle_long_.getBuffAngle(points_2d, 28.5, buff_angle_, angle_x_, angle_y_, distance_);
        angle_y_*=0.8;

#ifdef DEBUG_PLOT //0紫 1橙
        w_->addPoint(distance_, 0);
        //        w_->addPoint(angle_y_, 1);
        w_->plot();
#endif
        return 1;
    }
    return 0;
}

int BuffDetector::getDirection(float angle)
{
    float error_angle = last_angle_ - angle;
    //        cout << "error_angle" << error_angle << endl;
    last_angle_ = angle;
    if(fabs(error_angle) < max_filter_value_ && fabs(error_angle) < 1e-6f)
    {
        if(history_.size() < history_size_)
        {
            history_.push_back(error_angle);
        }else {
            history_.push_back(error_angle);
            history_.erase(history_.begin());
        }
    }
    std::vector<float>::iterator iter;
    float sum = 0.0;
    for (iter=history_.begin();iter!=history_.end();iter++){
        sum += *iter;
    }
    sum /= history_.size();
    //        cout << "sum " << sum << endl;

    if(sum >= 0.5)
        return 1;   // shun
    else if(sum <= 0.5)
        return -1;   // ni
    else
        return 0;
}


void Object::UpdateOrder()
{
    points_2d_.clear();
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
}

void Object::UpdataPredictPoint()
{
    float length_scale;
    float width_scale = width_scale_;
    if(direction_ < 0){
        length_scale = -length_scale_;
    }else
    {
        length_scale = length_scale_;
    }
    Point2f vector_length = points_2d_.at(0) - points_2d_.at(1);
    Point2f vector_width = points_2d_.at(0) - points_2d_.at(3);
    test_point_ = Point2f(points_2d_.at(1).x + vector_length.x * length_scale + vector_width.x * width_scale
                          , points_2d_.at(1).y + vector_length.y * length_scale + vector_width.y * width_scale);
    if(test_point_.x > 640)
        test_point_.x = 640;
    else if(test_point_.x < 0)
        test_point_.x = 0;
    if(test_point_.y > 360)  //industrial 480
        test_point_.y = 360;
    else if(test_point_.y < 0)
        test_point_.y = 0;
}

float Point_distance(Point2f p1,Point2f p2)
{
    float Dis=pow(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2),0.5);
    return Dis;
}

int AutoAttack::run(bool find_target_flag,float angle_x,float angle_y,int target_size)
{
    if(find_target_flag)
    {
        if(move_static==0)
        {
            control_=0;
        }
        else if(move_static==1)
        {
            if(target_size<4 || target_size ==5)
            {
                control_=0;
            }
            else if(target_size==4)
            {
                control_=1;
            }
        }

    }

    switch (control_)
    {
    case 0:
        if(find_target_flag)
        {
            if(fabs(angle_x) > 0.8f || fabs(angle_y) > 1.0f) // still not stable, wait
            {
                buff_mode=follow;
                t_tocul=0;
            }
            else if(fabs(angle_x) < 0.8f && fabs(angle_y) < 1.0f)
            {
                if(t_tocul==0) //stable, shoot
                {
                    buff_mode=shoot;
                    t_tocul++;
                }
                else if(t_tocul>0 && t_tocul <20) //only shoot once, then wait
                {
                    t_tocul++;
                    buff_mode=follow;
                }
                else if(t_tocul>20)  // when out of the time, back to center
                {
                    t_tocul=0;
                }
            }
        }
        else if(find_target_flag==0) // if don't find the target, back to center
        {
            buff_mode=restore_center;
        }
        break;

    case 1:
        if(find_target_flag)
        {
            if(fabs(angle_x) > 0.8f &&fabs(angle_y) > 1.0f && restore_count==0)
            {
                buff_mode=restore_center;
                ++restore_count;
            }
            else if (fabs(angle_x) > 0.8f &&fabs(angle_y) > 1.0f && restore_count!=0)
            {
                if(fabs(angle_x) > 0.8f || fabs(angle_y) > 1.0f)
                {
                    buff_mode=follow;
                    t_tocul=0;
                }
                else if(fabs(angle_x) < 0.8f && fabs(angle_y) < 1.0f)
                {
                    restore_count=0;
                    if(t_tocul==0)
                    {
                        buff_mode=shoot;
                        t_tocul++;
                    }
                    else if(t_tocul>0 && t_tocul <20)
                    {
                        t_tocul++;
                        buff_mode=follow;
                    }
                    else if(t_tocul>20)
                    {
                        t_tocul=0;
                    }
                }
            }
            else if(find_target_flag==0) // if don't find the target, back to center
            {
                buff_mode=restore_center;
            }
        }

        break;
    }
    return buff_mode;
}

