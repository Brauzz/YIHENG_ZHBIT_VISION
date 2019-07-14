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

double calcDistanceFor2Point(Point2f p1, Point2f p2)
{
    return pow(pow((p1.x-p2.x), 2) + pow((p1.y - p2.y), 2), 0.5);
}

/**
 * @brief conversionAbsolutePoint 对不同情况的点进行补偿
 * @param point_tmp 原始点
 * @param dst 最终处理
 * @param offset 补偿值
 * @param i1 转换序号
 * @param i2
 * @param i3
 * @param i4
 */
void conversionAbsolutePoint(Point2f *point_tmp, vector<Point2f>& dst
                             ,Point2f offset
                             ,int8_t i1, int8_t i2, int8_t i3, int8_t i4)
{
    dst.push_back(point_tmp[i1] + offset);
    dst.push_back(point_tmp[i2] + offset);
    dst.push_back(point_tmp[i3] + offset);
    dst.push_back(point_tmp[i4] + offset);
}



BuffDetector::BuffDetector()
{

}

BuffDetector::BuffDetector(SolveAngle solve_angle)
{
    solve_angle_long_ = solve_angle;
}

BuffDetector::~BuffDetector()
{

}

bool BuffDetector::DetectBuff(Mat& img)
{
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
    //TODO(cz): 建议使用拟合椭圆的方法重新实现
    RotatedRect small_target_rect;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary_color_img,contours,hierarchy,CV_RETR_CCOMP,CHAIN_APPROX_SIMPLE);
    for(size_t i=0; i < contours.size();i++)
    {
        if(hierarchy[i][3]<0)
            continue;
        double small_contour_area, big_contour_area;
        double small_contour_length, big_contour_length;
        small_contour_area = contourArea(contours[i]);
        if(small_contour_area < 5)
            continue;
        big_contour_area = contourArea(contours[static_cast<size_t>(hierarchy[i][3])]);
        if(big_contour_area < 5)
            continue;
        small_contour_length = arcLength(contours[i], true);
        if(small_contour_length < 5)
            continue;
        big_contour_length =  arcLength(contours[static_cast<size_t>(hierarchy[i][3])], true);
        if(big_contour_length < 5)
            continue;

        if(small_contour_area * 4       > big_contour_area
                && small_contour_area * 1.5 < big_contour_area
                && small_contour_length * 7 > big_contour_length
                && small_contour_length * 1.5 < big_contour_length)
        {
            // --debug--
            RotatedRect big_target_rect = minAreaRect(contours[static_cast<size_t>(hierarchy[i][3])]);
            Point2f big_points_tmp[4];
            big_target_rect.points(big_points_tmp);
            for(int j=0; j < 4; j++)
            {
                line(img, big_points_tmp[j], big_points_tmp[(j+1)%4], Scalar(255, 0, 255),1);
            }

            // --\debug--
            object small_target;
            small_target.rrect = minAreaRect(contours[i]);
            small_target.origin_rrect = small_target.rrect;
            Point2f points_tmp[4];
            small_target.rrect.points(points_tmp);
            Point2f origin_point_tmp[4];
            small_target.rrect.points(origin_point_tmp);
            circle(img, big_target_rect.center,2,Scalar(100,255,100));
            if(small_target.rrect.size.height < small_target.rrect.size.width)
            {
                small_target.rrect.angle += 90;
                float tmp = small_target.rrect.size.height;
                small_target.rrect.size.height = small_target.rrect.size.width;
                small_target.rrect.size.width = tmp;
            }

            // **目标数据转换** -用于计算世界坐标预测，计算绝对四个点，和计算角度
            if(small_target.rrect.size.height/small_target.rrect.size.width < 3.0f
                    && small_target.rrect.size.height/small_target.rrect.size.width > 1.0f)
            {
                for(int j=0; j < 4; j++)
                {
                    line(img, points_tmp[j], points_tmp[(j+1)%4], Scalar(0, 255, 255),1);
                }
                circle(img, small_target.rrect.center,2,Scalar(255,255,255));

                // ----------new alogrithm ----------------------------------
                float width_ = small_target.origin_rrect.size.width;
                float height_ = small_target.origin_rrect.size.height;
                Point2f big_target_center = big_target_rect.center;
                float offset_x = buff_offset_x_;
                float offset_y = buff_offset_y_;
                Point2f point_offset = Point2f(offset_x-100, offset_y-100);

                if(width_>=height_)
                {
                    // 上下装甲板边沿  此时装甲板水平和+角度
                    Point2f Lower_edge = (origin_point_tmp[0] + origin_point_tmp[3])/2.0f;
                    Point2f upper_edge = (origin_point_tmp[1] + origin_point_tmp[2])/2.0f;
                    //                    circle(img, Lower_edge, 10, Scalar(0,0,255),-1);
                    //                    circle(img, upper_edge, 10, Scalar(255,0,0),-1);
                    // 通过父轮廓中心判断中心靠近哪个边缘
                    float lower_edge_dist = calcDistanceFor2Point(Lower_edge, big_target_center);
                    float upper_edge_dist = calcDistanceFor2Point(upper_edge, big_target_center);
                    if( lower_edge_dist <= upper_edge_dist)
                    {
                        // 流水灯靠近下边沿
                        conversionAbsolutePoint(origin_point_tmp, points_2d, point_offset, 1, 2, 3, 0);
                        buff_angle_ = 90 - small_target.origin_rrect.angle;
                    }
                    else
                    {
                        // 流水灯靠近上边沿
                        conversionAbsolutePoint(origin_point_tmp, points_2d, point_offset, 3, 0, 1, 2);
                        buff_angle_ = 270 - small_target.origin_rrect.angle;
                    }
                }else
                {
                    // 左右装甲板边沿  此时装甲板垂直和-角度
                    Point2f left_edge = (origin_point_tmp[0] + origin_point_tmp[1])/2.0f;
                    Point2f right_edge = (origin_point_tmp[2] + origin_point_tmp[3])/2.0f;

                    // 通过父轮廓中心判断中心靠近哪个边缘
                    float left_edge_dist = calcDistanceFor2Point(left_edge, big_target_center);
                    float right_edge_dist = calcDistanceFor2Point(right_edge, big_target_center);
                    if(left_edge_dist <= right_edge_dist)
                    {
                        // 流水灯靠近左边沿
                        conversionAbsolutePoint(origin_point_tmp, points_2d, point_offset, 2, 3, 0, 1);
                        buff_angle_ =  - small_target.origin_rrect.angle;
                    }
                    else
                    {
                        // 流水灯靠近右边沿
                        conversionAbsolutePoint(origin_point_tmp, points_2d, point_offset, 0, 1, 2, 3);
                        buff_angle_ = 180 - small_target.origin_rrect.angle;
                    }
                }

                for(int l=0;l<4;l++)
                {
                    circle(img, points_2d.at(static_cast<size_t>(l)), 3, Scalar(0,255,255));
                    putText(img, to_string(l),
                            points_2d.at(static_cast<size_t>(l)),
                            FONT_HERSHEY_COMPLEX, 0.5,Scalar(255,255,255));
                }
                //-----------\new alorithm -----------------------------------
                return 1;
            }
        }
    }
    return 0;
}

int8_t BuffDetector::BuffDetectTask(Mat& img, OtherParam other_param)
{
    bool find_flag = DetectBuff(img);
    float theta_y = 0;
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
        solve_angle_long_.getAngle(points_2d, 28.5, angle_x_, angle_y_, distance_, theta_y);
        angle_y_*=0.8;
        INFO(distance_);
        return 1;
    }
    return 0;
}

int8_t BuffDetector::getDirection(float angle)
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

    if(sum >= 0)
        return 0;   // shun
    else
        return 1;   // ni
}
