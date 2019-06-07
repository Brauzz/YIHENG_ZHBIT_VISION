#include "armor_detect.h"

//#define DEBUG_DETECT
#define show_rect_angle
#define show_rect_bound
#define USE_FIT


double calc_distance(Point2f p1, Point2f p2)
{
    return pow(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2),0.5);
}


bool ArmorDetectTask(Mat &img, Param_ &param)
{
    vector<LED_Stick> LED_Stick_v;

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

    threshold(gray, binary_brightness_img, param.gray_th, 255, CV_THRESH_BINARY);
    threshold(result_img, binary_color_img, param.color_th, 255, CV_THRESH_BINARY);



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
#ifdef USE_FIT
                    RotatedRect RRect = fitEllipse( Mat(contours_brightness[i]));

#ifdef show_rect_bound

                    Point2f rect_point[4];
                    RRect.points(rect_point);
                    for (int i = 0; i < 3 ; i++)
                    {
                        line(img, Point_<int>(rect_point[i]), Point_<int>(rect_point[(i+1)%4]), Scalar(255,0,255),1);
                    }
#endif
                    if(RRect.angle>90.0f)
                        RRect.angle =  RRect.angle - 180.0f;
#ifdef show_rect_angle
                    //                    putText(img, to_string(RRect.angle), RRect.center + Point2f(2,2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
#endif
#else
                    RotatedRect RRect = minAreaRect( Mat(contours_brightness[i]));
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
                    if(RRect.size.height < RRect.size.width)    // convert angle to
                    {
                        RRect.angle+= 90;
                        double tmp = RRect.size.height;
                        RRect.size.height = RRect.size.width;
                        RRect.size.width = tmp;
                    }
#endif
                    if (abs(RRect.angle) <= 30)   // angle in range (-20,20) degree
                    {
                        LED_Stick r(RRect);
                        LED_Stick_v.push_back(r);


                    }


                }
                break;
            }
        }
    }

    for(size_t i = 0; i < LED_Stick_v.size() ; i++)
    {
        for(size_t j = i + 1; j < LED_Stick_v.size() ; j++)
        {
            armor arm_tmp( LED_Stick_v.at(i), LED_Stick_v.at(j) );
            if (arm_tmp.error_angle < 5.5f)
            {
                putText(img, to_string(int(arm_tmp.error_angle)), arm_tmp.center , FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
                if(arm_tmp.is_suitable_size())
                {
                    if(arm_tmp.get_average_intensity(gray)< 100 )
                    {
                        arm_tmp.max_match(LED_Stick_v, i, j);
//                                                    arm_tmp.draw_rect(img);
//                                                    arm_tmp.draw_spot(img);
                    }
                    //                    }

                }

            }
        }
    }
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

    // for draw
    float dist=1e8;
    armor target;
    target.center.x = 320 + param.offset_image.x -100; // trackbar get only positive vulue;
    target.center.y = 180 + param.offset_image.y -100;
    Point2f aim_center = target.center;
    float dx,dy;
    for (size_t i = 0; i < final_armor_list.size() ; i++ )
    {
        dx = pow((final_armor_list.at(i).center.x - aim_center.x), 2.0f);
        dy = pow((final_armor_list.at(i).center.y - aim_center.y), 2.0f);
        if( dx + dy < dist)
            target = final_armor_list.at(i);
                final_armor_list.at(i).draw_rect(img);
        target.draw_spot(img);

        Point2f point_tmp[4];
        Point2f point_2d[4];
        param.offset_image = Point2i(param.offset_x, param.offset_y);
        Point2f offset_point = Point2f(100, 100) - static_cast<Point2f>(param.offset_image);
        target.Led_stick[0].rect.points(point_tmp);

        point_2d[0] = point_tmp[1] + offset_point;
        point_2d[3] = point_tmp[0] + offset_point;
        target.Led_stick[1].rect.points(point_tmp);
        point_2d[1] = point_tmp[2] + offset_point;
        point_2d[2] = point_tmp[3] + offset_point;
        circle(img, point_2d[0],3,Scalar(255,255,255),1);
        circle(img, point_2d[1],3,Scalar(0,0,255),1);
        circle(img, point_2d[2],3,Scalar(0,255,0),1);
        circle(img, point_2d[3],3,Scalar(255,0,0),1);
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

void armor::draw_rect( Mat& img) const
{
    //        cout << " rect " << rect.width << "  " << rect.height << "　"<< rect.x << endl;
    rectangle(img, rect, Scalar(255,255,255), -1);
}

void armor::draw_spot(Mat &img) const
{
    circle(img, center, int(rect.height/4), Scalar(0,0,255), -1);
}

void armor::classification_lights()
{
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
    Led_stick[0].rect = L;
    Led_stick[1].rect = R;
}

int armor::get_average_intensity(const Mat& img) {
    if(rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y < 1 )
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
    Led_stick[0].rect = L;
    Led_stick[1].rect = R;
    float angle_8 = L.angle - R.angle;
//    cout << L.angle << " "<< R.angle << endl;
    if(angle_8 < 1e-3f)
        angle_8 = 0.0f;
    float f = error_angle + angle_8 /*+ abs(LED.at(i).rect.center.y - LED.at(j).rect.center.y)*/;
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
    if(Led_stick[0].rect.size.height*0.7f < Led_stick[1].rect.size.height
            && Led_stick[0].rect.size.height*1.3f > Led_stick[1].rect.size.height)
    {
        //        float h_max = Led_stick[0].rect.size.height > Led_stick[1].rect.size.height?
        //                    Led_stick[0].rect.size.height : Led_stick[1].rect.size.height;
        float h_max = (Led_stick[0].rect.size.height + Led_stick[1].rect.size.height)/2.0f;
        if(fabs(Led_stick[0].rect.center.y - Led_stick[1].rect.center.y) < 0.8f* h_max )
        {
            if(h_max*2.7f > rect.width && h_max < 2.0f* rect.width)
            {
                    return true;
            }
        }
    }
    return false;
}

