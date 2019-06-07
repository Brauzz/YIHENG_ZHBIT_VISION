#pragma once
#include <opencv2/opencv.hpp>
#include "settings.h"
using namespace std;
using namespace cv;
class LED_Stick{
public:

    LED_Stick():matched(false){}

    LED_Stick(const RotatedRect& R){
        rect.angle = R.angle;
        rect.center = R.center;
        rect.size = R.size;
        matched = false;
    }

    RotatedRect rect;
    bool matched;
    size_t match_index;
    float match_factor;
};


class armor{
public:
    armor();

    armor(const LED_Stick& L1, const LED_Stick& L2);

    void draw_rect( Mat& img) const;

    void draw_spot(Mat &img) const;

    int get_average_intensity(const Mat& img) ;

    void max_match(vector<LED_Stick>& LED, size_t i, size_t j);
    void classification_lights(void);
    bool is_suitable_size(void) const;

    LED_Stick Led_stick[2];
    float error_angle;
    Point2i center;
    Rect2i rect;
    int average_intensity;
};


class ArmorFilter {
public:
    ArmorFilter(int _filter_size = 5)
        : filter_size(_filter_size) {}

    void clear(){
        history.clear();
    }

    bool getResult(bool is_small){
        if (history.size() < filter_size){
            history.push_back(is_small);
        }
        else {
            history.push_back(is_small);
            history.pop_front();
        }

        int vote_cnt[2] = {0};
        for (std::list<bool>::const_iterator it = history.begin(); it != history.end(); ++it){
            *it == 0 ? ++vote_cnt[0] : ++vote_cnt[1];
        }

        if (vote_cnt[0] == vote_cnt[1])
            return is_small;
        return vote_cnt[0] > vote_cnt[1] ? 0 : 1;
    }

private:
    std::list<bool> history;
    int filter_size;
};


bool ArmorDetectTask(Mat &img, Param_ &param);

