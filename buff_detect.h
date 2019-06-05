#pragma once

#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;

class object
{
public:
    RotatedRect rrect;
    RotatedRect origin_rrect;
};



class DirectionFilter
{
public:
    DirectionFilter(int buff_size = 10, float filter_angle_threshold = 15)

    {
        history_size=buff_size;
        max_filter_value=filter_angle_threshold;
    }
    bool getDirection(float angle)
    {
        float error_angle = last_angle - angle;
//        cout << "error_angle" << error_angle << endl;
        last_angle = angle;
        if(abs(error_angle) < max_filter_value && error_angle != 0)
        {
            if(history.size() < history_size)
            {
                history.push_back(error_angle);
            }else {
                history.push_back(error_angle);
                history.erase(history.begin());
            }
        }
        std::vector<float>::iterator iter;
        float sum = 0.0;
        for (iter=history.begin();iter!=history.end();iter++){
            sum += *iter;
        }
        sum /= history.size();
//        cout << "sum " << sum << endl;

        if(sum >= 0)
            return 0;   // shun
        else
            return 1;   // ni

    }
private:
    vector<float> history;
    int history_size;
    float last_angle;
    float max_filter_value;
};


bool BuffDetectTask(Mat &img, vector<Point2f> &target_2d_point, int8_t our_color, Point2f offset_tvec, float &theta_angle);
float calcDistanceFor2Point(Point2f p1, Point2f p2);
void conversionAbsolutePoint(Point2f *point_tmp, vector<Point2f>& dst
                             ,Point2f offset
                             ,int8_t i1, int8_t i2, int8_t i3, int8_t i4);
