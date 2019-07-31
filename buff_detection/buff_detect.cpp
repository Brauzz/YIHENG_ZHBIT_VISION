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

    //    GaussianBlur(img, img, Size(3,3),0);
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
//        object.minArea_rect = minAreaRect(contours[i]);
//        object.Indeed_smallrect();
        object.big_rect_ = fitEllipse(contours[static_cast<uint>(hierarchy[i][3])]);
#else
        object.small_rect_=minAreaRect(contours[i]);
        object.big_rect_ = minAreaRect(contours[static_cast<uint>(hierarchy[i][3])]);
#endif

        //                drawContours(img,contours,i,Scalar(0,0,255),3);
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
        float diff_angle=fabsf(object.big_rect_.angle-object.small_rect_.angle);

        if(object.small_rect_.size.height/object.small_rect_.size.width < 3)
        {
            if(diff_angle<100 && diff_angle>80)
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
                float area_ratio = area_ratio_/100;
                if(small_rect_area * 12 >big_rect_area && small_rect_area* area_ratio<big_rect_area
                        && small_rect_size_ratio > 1 && small_rect_size_ratio < 3.0f)
                {
                    object.type_ = ACTION;  // 已经激活类型
                    //            putText(img, "ACTION", Point2f(20,20)+ object.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));
                }else if(small_rect_area * area_ratio>=big_rect_area && small_rect_area *2 < big_rect_area
                         && small_rect_size_ratio > 1 && small_rect_size_ratio < 3.0f)
                {

                    object.type_ = INACTION;    // 未激活类型
                    putText(img, "INACTION", Point2f(20,20)+ object.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));
                }else
                {
                    object.type_ = UNKOWN;    // 未激活类型
                    putText(img, "UNKOWN", Point2f(20,20)+ object.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));
                }

#ifdef AREA_LENGTH_ANGLE
                switch (AREA_LENGTH_ANGLE)
                {
                case 1:
                {
                    double multiple_area=fabs(big_rect_area/small_rect_area);
                    putText(img, to_string(multiple_area), Point2f(5,5)+ object.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));
                }break;
                case 2:
                {
                    double multiple_length=fabs(big_rect_length/small_rect_length);
                    putText(img, to_string(multiple_length), Point2f(5,5)+ object.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));
                }break;
                case 3:
                {
//                    putText(img, to_string(diff_angle), Point2f(20,20)+ object.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));
                }break;
                }

#endif
                // 更新世界坐标系顺序
                Point2f buff_offset = Point2f(100 - buff_offset_x_, 100 - buff_offset_y_);
                object.UpdateOrder();
                // 根据距离计算超预测点
                object.UpdataPredictPoint();
                circle(img , object.test_point_, 3, Scalar(22,255,25));
                vec_target.push_back(object);
#ifdef FUSION_MINAREA_ELLIPASE
            }
        }
#endif
    }
    // 遍历所有结果并处理\选择需要击打的目标
    //     TODO(cz): 超预测写了一版基础，未加入识别到5个激活目标后再进入超预测的逻辑，仅供参考
    Object final_target;
    vector<Object> inaction_target;
    bool find_flag = false;
    bool do_you_find_inaction = 1;
    // 你需要击打的能量机关类型 1(true)击打未激活 0(false)击打激活
    for(size_t i=0; i < vec_target.size(); i++)
    {

        Object object_tmp = vec_target.at(i);
        if(do_you_find_inaction==1)
        {
            //            trigger(target_size,direction_tmp);
            // 普通模式击打未激活机关
            if(object_tmp.type_ == INACTION)
            {

#ifdef DEBUG_PUT_TEST_TARGET
                //                putText(img, "<<---attack here"/*to_string(object_tmp.angle_)*/, Point2f(5,5)+ object_tmp.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
#endif
                inaction_target.push_back(object_tmp);


                find_flag = true;
            }
        }else if(do_you_find_inaction>=1)
        {
            //            trigger(target_size,direction_tmp);
            // 超预测模式击打目标选择
            bool is_contain = false;
            for(size_t j=0; j < vec_color_rect.size(); j++)
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
                putText(img, "<<---attack advance here", Point2f(5,5)+ object_tmp.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
#endif
                //                final_target object_tmp;
                points_2d=final_target.points_2d_;
                find_flag = true;
                break;
            }
        }

        //        putText(img, to_string(object_tmp.angle_), Point2f(5,5)+ object_tmp.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
    }

    //    for(size_t j=0; j < vec_color_rect.size(); j++)
    //    {
    //        Rect bound_rect_tmp = vec_color_rect.at(j);
    //        rectangle(img, bound_rect_tmp, Scalar(128,0,128));
    //    }
    if(find_flag == true){
        float dist = 1e8;
        float dx, dy;
        //        INFO(inaction_target.size());
        for(size_t i = 0; i< inaction_target.size(); i++)
        {
            // 计算补偿值

            dx = fabs(inaction_target.at(i).small_rect_.center.x - 320);
            dy = fabs(inaction_target.at(i).small_rect_.center.y - 240);
            if(dx + dy < dist)
            {
                final_target = inaction_target.at(i);
                dist = dx + dy;
            }
        }
        Point2f buff_offset = Point2f(100 - buff_offset_x_, 100 - buff_offset_y_);
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

#ifdef DEBUG_DRAW_TARGET

        final_target.DrawTarget(img);
        //        action_cnt_ = GetIndex(img, final_target, vec_color_rect);
    }
#endif
    return find_flag;
}


void makePointSafe(Point2f &point){
    if(point.x > 640)
        point.x = 640;
    else if(point.x < 0)
        point.x = 0;
    if(point.y > 480)  //industrial 480
        point.y = 480;
    else if(point.y < 0)
        point.y = 0;
}

int BuffDetector::GetIndex(Mat &img, Object object, vector<Rect> all_rect)
{
    if(object.angle_>45 && object.angle_<135)
    {
        return action_cnt_;
    }
    Point2f center = object.small_rect_.center;

    float length_scale = 1.1f;
    float width_scale = -5.5f;
    vector<Point2f> vec_point;

    Point2f length = object.points_2d_.at(0) - object.points_2d_.at(1);
    Point2f width = object.points_2d_.at(0) - object.points_2d_.at(3);
    Point2f test_point_ = Point2f(center.x + length.x * length_scale + width.x * width_scale
                                  , center.y + length.y * length_scale + width.y * width_scale);
    circle(img, test_point_, 5, Scalar(0,255,0),-1);
    putText(img, "1", test_point_, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
    vec_point.push_back(test_point_);

    length_scale = -1.1f;
    width_scale = -5.5f;
    length = object.points_2d_.at(0) - object.points_2d_.at(1);
    width = object.points_2d_.at(0) - object.points_2d_.at(3);
    test_point_ = Point2f(center.x + length.x * length_scale + width.x * width_scale
                          , center.y + length.y * length_scale + width.y * width_scale);
    circle(img, test_point_, 5, Scalar(0,255,0),-1);
    putText(img, "2", test_point_, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
    vec_point.push_back(test_point_);

    length_scale = -0.8f;
    width_scale = -8.0f;
    length = object.points_2d_.at(0) - object.points_2d_.at(1);
    width = object.points_2d_.at(0) - object.points_2d_.at(3);
    test_point_ = Point2f(center.x + length.x * length_scale + width.x * width_scale
                          , center.y + length.y * length_scale + width.y * width_scale);
    circle(img, test_point_, 5, Scalar(0,255,0),-1);
    putText(img, "3", test_point_, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
    vec_point.push_back(test_point_);

    length_scale = 0.8f;
    width_scale = -8.0f;
    length = object.points_2d_.at(0) - object.points_2d_.at(1);
    width = object.points_2d_.at(0) - object.points_2d_.at(3);
    test_point_ = Point2f(center.x + length.x * length_scale + width.x * width_scale
                          , center.y + length.y * length_scale + width.y * width_scale);
    circle(img, test_point_, 5, Scalar(0,255,0),-1);
    putText(img, "4", test_point_, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
    vec_point.push_back(test_point_);

    int cnt = 0;

    for(size_t i = 0; i < vec_point.size(); i++)
    {
        for(size_t j = 0; j < all_rect.size(); j++)
        {
            if(all_rect.at(j).contains(vec_point.at(i)) == true)
            {
                cnt ++;
                break;
            }
        }
    }
    if(action_cnt_ > cnt && action_cnt_ -2 < cnt){
        return action_cnt_;
    }
    else{
        return cnt;
    }


}
int BuffDetector::BuffDetectTask(Mat& img, OtherParam other_param)
{
    TIME_START(tt);
    color_ = other_param.color;
    gimbal=other_param.gimbal_data;
    bool find_flag = DetectBuff(img,other_param);
    int command = 0;
    if(find_flag)
    {
        find_cnt ++;
        if(find_cnt % 10==0)
        {
            //            direction_tmp = getDirection(buff_angle_);
            direction_tmp = getSimpleDirection(buff_angle_);
        }

        Point2f world_offset;
        //#define DIRECTION_FILTER
#ifdef DIRECTION_FILTER
        if(direction_tmp == 1)  // shun
            world_offset = Point2f(param_.world_offset_x  - 500, param_.world_offset_y - 500);
        else if(direction_tmp == -1)// ni
            world_offset = Point2f(-(param_.world_offset_x  - 500), -(param_.world_offset_y - 500));
        else
            world_offset = Point2f(world_offset_x_ - 500, world_offset_y_  - 500);
        cout << "direction " << direction_tmp << endl;

#else
        world_offset = Point2f(world_offset_x_ - 500, world_offset_y_  - 500);
#endif
        solve_angle_long_.Generate3DPoints(2, world_offset);
        solve_angle_long_.getBuffAngle(points_2d, 28.5, buff_angle_, angle_x_, angle_y_, distance_);

    }
    //    attack.run(find_flag,angle_x_,angle_y_,target_size,gimbal,direction_tmp);

    command = auto_control.run(angle_x_, angle_y_, find_flag);
#ifdef DEBUG_PLOT //0紫 1橙
    w_->addPoint(direction_tmp, 0);
    w_->addPoint(d_angle_, 1);
    w_->plot();
#endif

    return command;
}

int BuffDetector::getDirection(float angle)
{
    float error_angle =  angle - last_angle_;
    //            cout << "error_angle" << error_angle << endl;
    last_angle_ = angle;
    if(fabs(error_angle) < max_filter_value_ && fabs(error_angle) > 1e-6f)
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
    cout << "sum " << sum << endl;

    if(sum >= 0.5f)
        return 1;   // shun
    else if(sum <= 0.5f)
        return -1;   // ni
    else
        return 0;
}

int BuffDetector::getSimpleDirection(float angle)
{
    float error_angle = angle - last_angle_;
    last_angle_ = angle;
    if(fabs(error_angle) < 10 && fabs(error_angle) > 1e-6)
    {
        d_angle_ = (1 - r) * d_angle_ + r * error_angle;
    }

    if(d_angle_ > 2)
        return 1;
    else if(d_angle_ < -2)
        return -1;
    else
        return 0;
}


void Object::Indeed_smallrect()
{
    if(minArea_rect.size.width>=minArea_rect.size.height)
    {
        float temp=minArea_rect.size.width;
        minArea_rect.size.width=minArea_rect.size.height;
        minArea_rect.size.height=temp;
    }
    small_rect_.size.width=minArea_rect.size.width;
    small_rect_.size.height=minArea_rect.size.height;
    small_rect_.center=fitEllipse_rect.center;
    small_rect_.angle=fitEllipse_rect.angle;

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
    if(test_point_.y > 480)  //industrial 480
        test_point_.y = 480;
    else if(test_point_.y < 0)
        test_point_.y = 0;
}

double Point_distance(Point2f p1,Point2f p2)
{
    double Dis=pow(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2),0.5);
    return Dis;
}

int AutoAttack::run(bool find_target_flag, float angle_x, float angle_y, int target_size, float gimbal, int move_static)
{
    float diff_gimbal=fabsf(gimbal-20);
    adjust_control(find_target_flag,move_static,target_size);
    switch (control_)
    {
    case 0:
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

        break;

    case 1:
        float diff=fabs(gimbal-20);

        if(fabs(angle_x) > 0.8f &&fabs(angle_y) > 1.0f && restore_count==0)
        {
            buff_mode=restore_center;
            if(diff_gimbal<2)
            {++restore_count;}

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
                else if(t_tocul>20 && diff>2)
                {
                    buff_mode=restore_center;
                }
                else if(t_tocul>20 && diff <2)
                {
                    buff_mode=restore_center;
                    t_tocul=0;
                }
            }
        }
        break;
    }
    return buff_mode;
}

int AutoAttack::adjust_control(bool find_target_flag, int move_static,int target_size)
{
    if(find_target_flag)  //如果找到目标，选择击打模式
    {
        center_buff=0;
        if(move_static==0)
        {
            control_=0;
        }
        else if(abs(move_static)==1)
        {
            if(target_size==4)
            {
                control_=1;
            }
            else
            {
                control_=0;
            }
        }
    }
    else if(find_target_flag==0) // if don't find the target, back to center
    {
        if( center_buff==0)   //避免某一帧丢失目标
        {
            count_center++;
            if(count_center>=30)
                center_buff=1;
        }
        else if(center_buff==1)
        {
            count_center--;
            buff_mode=restore_center;
            if(count_center<0)
                count_center=0;
        }

    }
    return control_;
}

