#include "armor_detect.h"


//#define DEBUG_DETECT
//#define show_rect_angle
//#define show_rect_bound


/* armor ArmorDetectTask(Mat& img);
* @breif input the img output the target armor
* @param img input the image the capture get
* @return VecPoint2f the Armor led stick four point
* is order is top left, top right, bottom right, bootom left
* @author cz
* @date 2018.1.28
*/

bool ArmorDetectTask(Mat &img, Param_ &param)
{
    vector<LED_Stick> LED_Stick_v;
    double t1 = getTickCount();
    Mat binary_brightness_img, binary_color_img, gray;
        cvtColor(img,gray,COLOR_BGR2GRAY);
    //    Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
    //    dilate(img, img, element);
    vector<cv::Mat> bgr;
    split(img, bgr);
    Mat result_img;

    if(param.color == 0)
    {
        subtract(bgr[2], bgr[1], result_img);
    }else
    {
        subtract(bgr[0], bgr[2], result_img);
    }

    uint8_t show_bin_th = threshold(gray, binary_brightness_img, param.gray_th, 255, CV_THRESH_BINARY);
    uint8_t show_color_th = threshold(result_img, binary_color_img, param.color_th, 255, CV_THRESH_BINARY);



    //    printf("bin_th = %d, color_th = %d\r\n", show_bin_th, show_color_th);
#ifdef DEBUG_DETECT
        imshow("binary_brightness_img", binary_brightness_img);
        imshow("binary_color_img", binary_color_img);
#endif
    vector<vector<Point>> contours_light;
    vector<vector<Point>> contours_brightness;
    findContours(binary_color_img, contours_light, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    findContours(binary_brightness_img, contours_brightness, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    for(size_t i = 0; i < contours_brightness.size(); i++)
    {
        double area = contourArea(contours_brightness[i]);
        if (area < 20.0 || 1e5 < area) continue;
        for(size_t ii = 0; ii < contours_light.size(); ii++)
        {
            if(pointPolygonTest(contours_light[ii], contours_brightness[i][0], false) >= 0.0 )
            {
                double length = arcLength( Mat(contours_brightness[i]), true);
                if (length > 30 && length <400)
                {
                    RotatedRect RRect = minAreaRect( Mat(contours_brightness[i]));
                    if(RRect.size.height < RRect.size.width)    // convert angle to
                    {
                        RRect.angle+= 90;
                        double tmp = RRect.size.height;
                        RRect.size.height = RRect.size.width;
                        RRect.size.width = tmp;
                    }

                    if (abs(RRect.angle) <= 30)   // angle in range (-20,20) degree
                    {
                        LED_Stick r(RRect);
                        LED_Stick_v.push_back(r);

                        #ifdef show_rect_angle
                        putText(img, to_string(int(RRect.angle)), RRect.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 2);
                        #endif
                        #ifdef show_rect_bound

                        Point2f rect_point[4];
                        RRect.points(rect_point);
                        for (int i = 0; i < 3 ; i++)
                        {
                            line(img, Point_<int>(rect_point[i]), Point_<int>(rect_point[(i+1)%4]), Scalar(255,0,255),2);
                        }
                        #endif
                    }
                }
                break;
            }
        }
    }

    for(int i = 0; i < LED_Stick_v.size() ; i++)
    {
        #pragma omp for
        for(int j = i + 1; j < LED_Stick_v.size() ; j++)
        {
            armor arm_tmp( LED_Stick_v.at(i), LED_Stick_v.at(j) );
            if (abs(arm_tmp.error_angle) < 8)
            {
                if(arm_tmp.is_suitable_size())
                {
                    if(arm_tmp.get_average_intensity(gray)< 100 )
                    {
                        arm_tmp.max_match(LED_Stick_v, i, j);
//                            arm_tmp.draw_rect(img);
//                            arm_tmp.draw_spot(img);
                    }
                }
            }
        }
    }
    vector<armor> final_armor_list;
    for(int i = 0; i < LED_Stick_v.size() ; i++)
    {
        if(LED_Stick_v.at(i).matched)
        {
            LED_Stick_v.at(LED_Stick_v.at(i).match_index).matched = false; //clear another matching flag
            armor arm_tmp( LED_Stick_v.at(i), LED_Stick_v.at(LED_Stick_v.at(i).match_index));
            final_armor_list.push_back(arm_tmp);
        }
    }

    // for draw
    float dist=1e8;
    armor target;
    target.center.x = 320 + param.offset_image.x -100; // trackbar get only positive vulue;
    target.center.y = 180 + param.offset_image.y -100;
    Point2f aim_center = target.center;
    float dx,dy;
    for (int i = 0; i < final_armor_list.size() ; i++ )
    {
        dx = pow((final_armor_list.at(i).center.x - aim_center.x), 2);
        dy = pow((final_armor_list.at(i).center.y - aim_center.y), 2);
        if( dx + dy < dist)
            target = final_armor_list.at(i);
        final_armor_list.at(i).draw_rect(img);
        target.draw_spot(img);
        RotatedRect right, left;
        if(target.Led_stick[0].rect.center.x > target.Led_stick[1].rect.center.x)
        {
            right = target.Led_stick[0].rect;
            left = target.Led_stick[1].rect;
        }else
        {
            right = target.Led_stick[1].rect;
            left = target.Led_stick[0].rect;
        }
        Point2f point_tmp[4];
        Point2f point_2d[4];
        param.offset_image = Point2f(param.offset_x, param.offset_y);
        Point2f offset_point = Point2f(100, 100) - param.offset_image;
        left.points(point_tmp);
        point_2d[0] = point_tmp[1] + offset_point;
        point_2d[3] = point_tmp[0] + offset_point;
        right.points(point_tmp);
        point_2d[1] = point_tmp[2] + offset_point;
        point_2d[2] = point_tmp[3] + offset_point;

        param.points_2d.clear();
        for(int i=0;i<4;i++)
        {
            param.points_2d.push_back(point_2d[i]);
        }
        return 1;
    }



    return 0;
}




bool ArmorDetectTask1(Mat &img, Param_ &param)
{

    double t1 = getTickCount();
    Mat binary_brightness_img, binary_color_img, gray;

    //    Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
    //    dilate(img, img, element);
    vector<cv::Mat> bgr;
    split(img, bgr);
    Mat result_img;

    if(param.color == 0)
    {
        subtract(bgr[2], bgr[1], result_img);
    }else
    {
        subtract(bgr[0], bgr[2], result_img);
    }
#if(1)
    cvtColor(img,gray,COLOR_BGR2GRAY);
    double show_bin_th = threshold(gray, binary_brightness_img, param.gray_th, 255, CV_THRESH_BINARY);
#else
    uint8_t show_bin_th = threshold(bgr[2], binary_brightness_img, param.gray_th, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
#endif
    double show_color_th = threshold(result_img, binary_color_img, param.color_th, 255, CV_THRESH_BINARY);


    //    printf("bin_th = %d, color_th = %d\r\n", show_bin_th, show_color_th);
#ifdef DEBUG_DETECT
    imshow("binary_brightness_img", binary_brightness_img);
    imshow("binary_color_img", binary_color_img);
#endif

    vector<vector<Point>> contours_light;
    vector<vector<Point>> contours_brightness;
    vector<vector<Point>> contours;
    findContours(binary_color_img, contours_light, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    findContours(binary_brightness_img, contours_brightness, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    for(size_t i = 0; i < contours_brightness.size(); i++)
    {
        double area = contourArea(contours_brightness[i]);
        if (area < 20.0 || 1e5 < area) continue;
        for(size_t ii = 0; ii < contours_light.size(); ii++)
        {
            if(pointPolygonTest(contours_light[ii], contours_brightness[i][0], false) >= 0.0 )
            {
                contours.push_back(contours_brightness[i]);
            }
        }
    }

    RotatedRect RA[16], R[16];
    int hi = 0;

    for(size_t i = 0; i < contours.size(); i++)
    {
        vector<Point> points;
        points = contours[i];
        RotatedRect rrect = fitEllipse(points);
        cv::Point2f* vertices = new cv::Point2f[4];
        rrect.points(vertices);

        for(int j = 0; j < 4; j++)
        {
            line(img,vertices[j],vertices[(j+1)%4], Scalar(0, 255, 0), 2);
        }

        double high = static_cast<double>(rrect.size.height);

        for(size_t j = 1; j < contours.size(); j++)
        {
            vector<Point> pointsA;
            double area = contourArea(contours[j]);
            if(area < 20 || 1e5 < area) continue;

            pointsA = contours[j];
            RotatedRect rrectA = fitEllipse(pointsA);


            double max_height, min_height;
            if(rrect.size.height > rrectA.size.height)
            {
                max_height = static_cast<double>(rrect.size.height);
                min_height = static_cast<double>(rrectA.size.height);
            }
            else
            {
                max_height = static_cast<double>(rrectA.size.height);
                min_height = static_cast<double>(rrect.size.height);
            }

            double lights_angle_error = static_cast<double>(abs(rrect.angle - rrectA.angle));
            double highA = static_cast<double>(rrectA.size.height);
            double lights_center_distance = sqrt((rrect.center.x - rrectA.center.x)*(rrect.center.x - rrectA.center.x)
                                                 + (rrect.center.y - rrectA.center.y)*(rrect.center.y - rrectA.center.y));
            double lights_x_distance = abs(rrect.center.x - rrectA.center.x);
            double armor_wh_ratio = lights_center_distance/((highA+high)/2);     // judge big armor and small armor
            double lights_height_diff = max_height - min_height;
            double lights_width_diff = abs(rrect.size.width - rrectA.size.width);
            double lights_avg_height = (rrect.size.height + rrectA.size.height)/200;
            double lights_avg_angle = abs(rrect.angle + rrectA.angle)/2;
            if((armor_wh_ratio < 3.0 - lights_avg_height && armor_wh_ratio > 2.0 - lights_avg_height
                && lights_angle_error <= 5.0 && lights_height_diff <= 8.0 && lights_width_diff <=5.0
                && (lights_avg_angle <= 30 || lights_avg_angle >= 150.0) && lights_x_distance > 0.6*lights_height_diff)
                    || (armor_wh_ratio < 5.0 - lights_avg_height && armor_wh_ratio > 3.2 - lights_avg_height
                        && lights_angle_error <= 7.0 && lights_height_diff <= 15.0 && lights_width_diff <= 8.0
                        && (lights_avg_angle <= 30.0 || lights_avg_angle >= 150.0) && lights_x_distance > 0.7*lights_height_diff))
            {
                R[hi] = rrect;
                RA[hi] = rrectA;
                hi++;
            }
        }
    }
    double min = 666;
    int mark = 0;
    for(int i = 0;i < hi;i++){     //多个目标存在，打center更近装甲板
        float dist = sqrt(pow(320-(R[i].center.x+RA[i].center.x)/2.0f,2)+pow(180-(R[i].center.y+RA[i].center.y)/2.0f,2));
        if(dist < min)
        {
            mark = i;
            min = dist;
        }
    }
    if(hi != 0){
        cv::circle(img,Point((R[mark].center.x+RA[mark].center.x)/2,
                             (R[mark].center.y+RA[mark].center.y)/2),
                   15,cv::Scalar(0,0,255),4);
        RotatedRect right, left;
        if(R[mark].center.x > RA[mark].center.x)
        {
            right = R[mark];
            left = RA[mark];
        }else
        {
            right = RA[mark];
            left = R[mark];
        }
        Point2f point_tmp[4];
        Point2f point_2d[4];
        left.points(point_tmp);
        point_2d[0] = point_tmp[1] + param.offset_image;
        point_2d[3] = point_tmp[0] + param.offset_image;
        right.points(point_tmp);
        point_2d[1] = point_tmp[2] + param.offset_image;
        point_2d[2] = point_tmp[3] + param.offset_image;

        param.points_2d.clear();
        for(int i=0;i<4;i++)
        {
            param.points_2d.push_back(point_2d[i]);
        }
        return 1;
    }
    return 0;
}
armor::armor(){}

armor::armor(const LED_Stick& L1, const LED_Stick& L2){
    Led_stick[0]= L1;
    Led_stick[1]= L2;
    error_angle = L1.rect.angle - L2.rect.angle;

    rect.width = abs(L1.rect.center.x - L2.rect.center.x);
    rect.height = (L1.rect.size.height + L1.rect.size.height)/2;
    center.x = (L1.rect.center.x + L2.rect.center.x)/2;
    center.y = (L1.rect.center.y + L2.rect.center.y)/2;
    rect.x = center.x - rect.width/3;
    rect.y = center.y - rect.height/3;
    rect.width*= 2.0/3;
    rect.height*= 2.0/3;
}

void armor::draw_rect( Mat& img) const
{
//        cout << " rect " << rect.width << "  " << rect.height << "　"<< rect.x << endl;
    rectangle(img, rect, Scalar(255,255,255), -1);
}

void armor::draw_spot(Mat &img) const
{
    circle(img, center, int(rect.height/4), Scalar(0,0,255), -1);
}

int armor::get_average_intensity(const Mat& img) {
    if(rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y < 1 )
        return 255;
    Mat roi = img(Range(rect.y, rect.y + rect.height), Range(rect.x, rect.x + rect.width) );
//        imshow("roi ", roi);
    average_intensity = mean(roi).val[0];
    return average_intensity;
}

void armor::max_match(vector<LED_Stick>& LED,const int& i, const int& j){
    float f = error_angle + abs(LED.at(i).rect.center.y - LED.at(j).rect.center.y);
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
    if(Led_stick[0].rect.size.height*0.6 < Led_stick[1].rect.size.height
            && Led_stick[0].rect.size.height*1.4 > Led_stick[1].rect.size.height)
    {

        int h_max = Led_stick[0].rect.size.height > Led_stick[1].rect.size.height?
                    Led_stick[0].rect.size.height : Led_stick[1].rect.size.height;
        if(abs(Led_stick[0].rect.center.y - Led_stick[1].rect.center.y) < 0.8* h_max ){
                if(h_max*3.6 > rect.width && h_max < rect.width )
                    return true;
        }
    }
    return false;
}

