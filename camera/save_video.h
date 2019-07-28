#pragma once
#include <opencv2/opencv.hpp>

using namespace cv;

class SaveVideo
{
public:
    SaveVideo();
    ~SaveVideo();
    void updateImage(Mat img);
    bool getState(){
        return state_;
    }
private:
    VideoWriter video_writer_;
    bool state_ = false;
};
