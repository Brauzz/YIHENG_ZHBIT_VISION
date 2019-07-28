#include "save_video.h"


SaveVideo::SaveVideo()
{
    struct tm *newtime;
    char tmpbuf[128];
    time_t test;
    time(&test);
    newtime=localtime(&test);
    strftime(tmpbuf, 128, "%c", newtime);
    sprintf(tmpbuf, "%s.avi", tmpbuf);
    video_writer_.open(tmpbuf, CV_FOURCC('X', 'V', 'I', 'D'), 60, cv::Size(640, 480));

    if (!video_writer_.isOpened()) {
        std::cout << "videowriter opened failure!" << std::endl;
        state_ = false;
    }else
    {
        state_ = true;
    }
}

SaveVideo::~SaveVideo()
{
    video_writer_.release();
}

void SaveVideo::updateImage(Mat img)
{
    video_writer_ << img;
}

