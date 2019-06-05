#pragma once
#include <opencv2/opencv.hpp>
using namespace cv;

class CaptureVideo
{
public:
    CaptureVideo(const char* device, unsigned int in_size_buffer = 1);
    ~CaptureVideo();
    bool startStream();
    bool closeStream();
    char* video_name(char* format =".avi");

    // setting
    bool setExposureTime(bool in_exp, int in_t);
    bool setVideoFormat(int in_width, int in_height, bool in_mjpg = 1);
    bool setVideoFPS(int in_fps);
    bool setBufferSize(int in_buffer_size);

    // restarting
    bool changeVideoFormat(int in_width, int in_height, bool in_mjpg = 1);
    void restartCapture();

    // getting
    bool getVideoSize(int &width, int &height);
    int getFrameCount(){
        return current_frame;
    }
    void info();

    CaptureVideo& operator >> (Mat & image);

private:
    void cvtRaw2Mat(const void * data, Mat &image);
    bool refreshVideoFormat();
    bool initMMap();
    int xioctl(int in_fd,int in_request, void *arg);

    struct MapBuffer
    {
        void * ptr;
        unsigned int size;
    };
    unsigned int capture_width;
    unsigned int capture_height;
    unsigned int format;
    int fd;
    unsigned int buffer_size;
    unsigned int buffer_index;
    unsigned int current_frame;
    MapBuffer * mb;
    const char * video_path;

};
