#include "base_detect.h"

Mat BaseDetector::setImage(const Mat &src)
{
    Rect rect_roi;
    Mat result_mat;
    const cv::Point2f last_result_center = last_target.center;
    if(last_result_center.x == 0 || last_result_center.y == 0)
    {
        rect_roi = Rect(0, 0, src.cols, src.rows);
    }
    else
    {
        Rect rect = last_target.boundingRect();
        int max_half_w = 1280;
        int max_half_h = 1024;
        float scale = 2.5;
        //        if (lost_cnt < 3)
        //            scale = 1.2;
        //        else if(lost_cnt == 6)
        //            scale = 1.5;
        //        else if(lost_cnt == 12)
        //            scale = 2.0;
        int exp_half_w = min(max_half_w / 2, int(rect.width * scale));
        int exp_half_h = min(max_half_h / 2, int(rect.height * 2* scale));

        int w = std::min(max_half_w, exp_half_w);
        int h = std::min(max_half_h, exp_half_h);
        Point center = last_result_center;
        int x = std::max(center.x - w, 0);
        int y = std::max(center.y - h, 0);
        Point lu = Point(x, y);
        x = std::min(center.x + w, src.cols);
        y = std::min(center.y + h, src.rows);
        Point rd = Point(x, y);
        //        cout << lu << endl;
        detect_rect_ = Rect(lu, rd);
        if(makeRectSafe(detect_rect_, src.size())== false)
        {
            last_target = cv::RotatedRect();
            detect_rect_ = Rect(0, 0, src.cols, src.rows);

        }
        rect_roi = detect_rect_;
    }
    src(rect_roi).copyTo(result_mat);
    return result_mat;
}

bool BaseDetector::detectBase(Mat &img)
{
    //    GaussianBlur(img , img, Size(3, 3),0,0);
    Mat binary_brightness_img, binary_color_img, gray;
    cvtColor(img,gray,COLOR_BGR2GRAY);
    //    Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
    //    dilate(img, img, element);
    vector<cv::Mat> bgr;
    split(img, bgr);
    Mat result_img;
    if(/* DISABLES CODE */ (1))
    {
        subtract(bgr[2], bgr[1], result_img);
    }else
    {
        subtract(bgr[0], bgr[2], result_img);
    }
    threshold(gray, binary_brightness_img, gray_th_, 255, CV_THRESH_BINARY);
    threshold(result_img, binary_color_img, color_th_, 255, CV_THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_RECT, Size(9,9));
//    morphologyEx(binary_brightness_img, binary_brightness_img, MORPH_CLOSE, element);

    // **提取可能的灯条** -利用灯条（灰度）周围有相应颜色的光圈包围
    //    printf("bin_th = %d, color_th = %d\r\n", show_bin_th, show_color_th);
#ifdef DEBUG_ARMOR_DETECT
//    imshow("binary_brightness_img", binary_brightness_img);
    imshow("binary_color_img", binary_color_img);
#endif
    vector<vector<Point>> contours_light;
    vector<vector<Point>> contours_brightness;
    vector<LightStick> vec_light_stick;
    findContours(binary_color_img, contours_light, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    findContours(binary_brightness_img, contours_brightness, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    //#pragma omp for
#if(1)
    for(size_t i = 0; i < contours_brightness.size(); i++)
    {

        double area = contourArea(contours_brightness[i]);
        if (area < 35.0 || 100 < area) continue;
#pragma omp for
        for(size_t ii = 0; ii < contours_light.size(); ii++)
        {
            //            test_cnt ++;
            if(pointPolygonTest(contours_light[ii], contours_brightness[i][0], false) >= 0.0 )
            {
                double length = arcLength(contours_brightness[i], true); // 灯条周长
                if (length > 25 && length <60)
                {
                    // 使用拟合椭圆的方法要比拟合最小矩形提取出来的角度更精确
                    RotatedRect RRect = fitEllipse(contours_brightness[i]);
#ifdef show_rect_bound
                    // 旋转矩形提取四个点
                    Point2f rect_point[4];
                    RRect.points(rect_point);
                    for (int i = 0; i < 4 ; i++)
                    {
                        line(img, Point_<int>(rect_point[i]), Point_<int>(rect_point[(i+1)%4]), Scalar(255,0,255),1);
                    }
#endif
                    // 角度换算，将拟合椭圆0~360 -> -180~180
                    //                    if(RRect.angle>90.0f)
                    //                        RRect.angle =  RRect.angle - 180.0f;
#ifdef show_rect_angle
                    putText(img, to_string(static_cast<int>(length)), RRect.center + Point2f(2,-50), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
#endif
                    LightStick led_stick(RRect);
                    vec_light_stick.push_back(led_stick);
                }
                break;
            }
        }
    }
#else
    for(size_t i = 0; i < contours_light.size(); i++)
    {

        double area = contourArea(contours_light[i]);
        if (area < 30.0 || 1e5 < area) continue;
        double length = arcLength(contours_light[i], true); // 灯条周长
        if (length > 30 && length <200)
        {
            // 使用拟合椭圆的方法要比拟合最小矩形提取出来的角度更精确
            RotatedRect RRect = fitEllipse(contours_light[i]);
#ifdef show_rect_bound
            // 旋转矩形提取四个点
            Point2f rect_point[4];
            RRect.points(rect_point);
            for (int k = 0; k < 4 ; k++)
            {
                line(img, rect_point[k] + Point2f(detect_rect_.x, detect_rect_.y), rect_point[(k+1)%4]+Point2f(detect_rect_.x, detect_rect_.y), Scalar(255,0,255),1);
            }
#endif
#ifdef show_rect_angle
            putText(img, to_string(static_cast<int>(RRect.angle)), RRect.center + Point2f(detect_rect_.x, detect_rect_.y) + Point2f(2,-50), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
#endif
            LightStick led_stick(RRect);
            vec_light_stick.push_back(led_stick);
        }
    }
#endif
    vector<BaseArmor> v_base_armor;
    for(int i = 0; i < vec_light_stick.size(); i++)
    {
        for(int j = i + 1; j < vec_light_stick.size(); j++)
        {
            BaseArmor armor_tmp(vec_light_stick.at(i), vec_light_stick.at(j));

            if(armor_tmp.is_suitable_size())
            {
                v_base_armor.push_back(armor_tmp);
            }
        }
    }

    BaseArmor target_armor;
    bool find_flag = false;
    RotatedRect target_rect;
    if(v_base_armor.size())
    {
        target_armor = v_base_armor.at(0);
        target_armor.draw_rect(img, points_2d_,Point2f(0,0)/*DRAW_OFFSET*/);
//        target_rect = minAreaRect(points_2d_);

        //        Rect draw_rect;
        //        draw_rect = rect_tmp.boundingRect();
        //        rectangle(img , draw_rect, Scalar(255, 255, 255));
        find_flag = true;
        for(int i = 0; i < 4; i++)
        {
            points_2d_.at(i) = points_2d_.at(i) + Point2f(offset_x_ - 100, offset_y_ - 100)
                    /*+ Point2f(detect_rect_.x, detect_rect_.y)*/;
            circle(img, points_2d_.at(i),3,Scalar(0,255,255));
        }
    }else {
        points_2d_.clear();
    }

//    rectangle(img, detect_rect_, Scalar(255, 0, 255), 3);
//    if(find_flag){
//        target_rect.center.x += detect_rect_.x;
//        target_rect.center.y += detect_rect_.y;
//        last_target = target_rect;
//        lost_cnt = 0;
//    }
//    else{
//        ++lost_cnt;
//        int scale = 5;
//        if (lost_cnt == 6*scale)
//            last_target.size =Size2f(last_target.size.width * 1.5, last_target.size.height * 1.5);
//        else if(lost_cnt == 12*scale)
//            last_target.size =Size2f(last_target.size.width * 2.0, last_target.size.height * 2.0);
//        else if(lost_cnt == 24*scale)
//            last_target.size =Size2f(last_target.size.width * 2.5, last_target.size.height * 2.5);
//        else if(lost_cnt == 40*scale)
//            last_target.size =Size2f(last_target.size.width * 3.0, last_target.size.height * 3.0);
//        else if (lost_cnt > 60*scale )
//            last_target = RotatedRect();

//    }

    return find_flag;
}



int8_t BaseDetector::BaseDetectTask(Mat &img)
{
    int8_t command = 0;
    command = detectBase(img);
    if(command == 1)
    {
        solve_angle_.Generate3DPoints(3, Point2f(-120, 0));
        solve_angle_.getAngle(points_2d_, 28, angle_x_, angle_y_, distance_, theta_y_);
        putText(img, "yaw :" + to_string(angle_x_) + " pitch :" + to_string(angle_y_) + " distacne: " + to_string(distance_), Point2f(20,20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
    }
    if(distance_ < 8000)
        command = 0;
    return command;
}

BaseArmor::BaseArmor(const LightStick &L1, const LightStick &L2)
{
    if(L1.rect_.center.x < L2.rect_.center.x)
    {
        left_stick_ = L1;
        right_stick_ = L2;
    }else
    {
        left_stick_ = L2;
        right_stick_ = L1;
    }
    center_.x = static_cast<int>((L1.rect_.center.x + L2.rect_.center.x)/2);
    center_.y = static_cast<int>((L1.rect_.center.y + L2.rect_.center.y)/2);
}

bool BaseArmor::is_suitable_size() const
{

    float left_angle = left_stick_.rect_.angle;
    float right_angle = right_stick_.rect_.angle;
    //    cout << left_angle << " "<< right_angle << endl;
    if(left_angle > 20.0f && left_angle < 100.0f
            && right_angle > 90.0f && right_angle < 170.0f)
    {
        float left_height = left_stick_.rect_.size.height;
        float right_heigth = right_stick_.rect_.size.height;
        if(left_height * 0.7f < right_heigth && left_height * 1.4f > right_heigth)
        {
            float width = fabs(left_stick_.rect_.center.x - right_stick_.rect_.center.x);
            float avg_height = (left_height + right_heigth)/2.0f;
            if(width * 1.0 > avg_height && width*0.3 < avg_height)
            {
                float angle_ratio = left_angle / right_angle;

                if(angle_ratio > 0.2f && angle_ratio < 0.8f)
                {
                    return 1;
                }
            }
        }

    }
    return 0;
}

void BaseArmor::draw_rect(Mat &img, vector<Point2f> &points_2d, Point2f offset_point) const
{
    Point2f left_points[4];
    left_stick_.rect_.points(left_points);
    Point2f right_points[4];
    right_stick_.rect_.points(right_points);
    vector<Point2f> v_rect;
    v_rect.push_back(left_points[2]);
    v_rect.push_back(right_points[3]);
    v_rect.push_back(right_points[2]);
    v_rect.push_back(left_points[3]);
    points_2d.clear();
    points_2d = v_rect;
    for(int i=0; i< 4; i++)
    {
        putText(img, to_string(i), v_rect.at(i) /*+offset_point*/+ Point2f(2,2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
        line(img, v_rect.at(i)+offset_point, v_rect.at((i+1)%4)+offset_point, Scalar(255, 255, 255));
    }
}
