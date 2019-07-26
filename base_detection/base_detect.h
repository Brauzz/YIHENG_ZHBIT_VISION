#pragma once
#include <opencv2/opencv.hpp>
#include "../common/solve_angle/solve_angle.h"
using namespace std;
using namespace cv;

#define show_rect_bound
#define show_rect_angle
//#define DEBUG_ARMOR_DETECT
#define DRAW_OFFSET Point2f(detect_rect_.x, detect_rect_.y)
class LightStick{
public:
    LightStick():matched_(false){}
    LightStick(const RotatedRect &R)
    {
        rect_.angle = R.angle;
        rect_.center = R.center;
        rect_.size = R.size;
        matched_ = false;
    }
    RotatedRect rect_;
    bool matched_;
    size_t match_index_;
    float match_factor_;
};

class BaseArmor{
public:
    BaseArmor(){}
    BaseArmor(const LightStick& L1, const LightStick& L2);
    bool is_suitable_size(void) const;
    void draw_rect(Mat& img, vector<Point2f> &points_2d, Point2f offset_point) const;
    LightStick left_stick_;
    LightStick right_stick_;
    Point2f center_;
    RotatedRect rect_;
};

class BaseDetector{
public:
    BaseDetector(){
        solve_angle_ = SolveAngle(CAMERA1_FILEPATH, LONG_X, LONG_Y, LONG_Z, PTZ_TO_BARREL);
    }
    ~BaseDetector(){}

    int8_t BaseDetectTask(Mat &img);
    void getAngle(float &yaw, float &pitch)
    {
        yaw = angle_x_;
        pitch = angle_y_;
    }

private:
    Mat setImage(const cv::Mat &src);
    bool detectBase(Mat &img);

private:
    bool makeRectSafe(cv::Rect & rect, cv::Size size){
        if (rect.x < 0)
            rect.x = 0;
        if (rect.x + rect.width > size.width)
            rect.width = size.width - rect.x;
        if (rect.y < 0)
            rect.y = 0;
        if (rect.y + rect.height > size.height)
            rect.height = size.height - rect.y;
        if (rect.width <= 0 || rect.height <= 0)
            return false;
        return true;
    }

public:
    SolveAngle solve_angle_;
    int gray_th_ = 39;
    int color_th_ = 50;
    vector<Point2f> points_2d_;
    float angle_x_;
    float angle_y_;
    float distance_;
    float theta_y_;

    int offset_x_ = 242;
    int offset_y_ = 197;
public:
    RotatedRect last_target;
    Rect detect_rect_;
    int lost_cnt = 0;
};
