#include "capture_video.h"
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
CaptureVideo::CaptureVideo(const char* device, unsigned int in_size_buffer):video_path(device)
{
    fd = open(device, O_RDWR);
    buffer_size = in_size_buffer;
    buffer_index = 0;
    current_frame = 0;
    capture_width = 0;
    capture_height = 0;
    mb = new MapBuffer[buffer_size];
}

CaptureVideo::~CaptureVideo()
{
    close(fd);
    delete[] mb;
}

bool CaptureVideo::startStream()
{
    current_frame = 0;
    refreshVideoFormat();
    if(initMMap() == false)
        return false;

    __u32 type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(fd, VIDIOC_STREAMON, &type) < 0){
        perror("VIDIOC_STREAMON");
        return false;
    }
    return true;
}

bool CaptureVideo::closeStream()
{
    current_frame = 0;
    buffer_index = 0;
    __u32 type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(fd, VIDIOC_STREAMOFF, &type) < 0){
        perror("VIDIOC_STREAMOFF");
        return false;
    }
    for(int i = 0; i < buffer_size; ++i){
        munmap(mb[i].ptr, mb[i].size);
    }
    return true;
}

void CaptureVideo::info()
{
    struct v4l2_capability caps = {};
    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps)) {
            perror("Querying Capabilities\n");
            return;
    }

    printf( "Driver Caps:\n"
            "  Driver: \"%s\"\n"
            "  Card: \"%s\"\n"
            "  Bus: \"%s\"\n"
            "  Version: %d.%d\n"
            "  Capabilities: %08x\n",
            caps.driver,
            caps.card,
            caps.bus_info,
            (caps.version>>16)&&0xff,
            (caps.version>>24)&&0xff,
            caps.capabilities);


    struct v4l2_cropcap cropcap = {0};
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl (fd, VIDIOC_CROPCAP, &cropcap))  {
            perror("Querying Cropping Capabilities\n");
            return;
    }

    printf( "Camera Cropping:\n"
            "  Bounds: %dx%d+%d+%d\n"
            "  Default: %dx%d+%d+%d\n"
            "  Aspect: %d/%d\n",
            cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
            cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
            cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);

    struct v4l2_fmtdesc fmtdesc = {0};
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    char fourcc[5] = {0};
    char c, e;
    printf("  FMT : CE Desc\n--------------------\n");
    while (0 == xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
            strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
            c = fmtdesc.flags & 1? 'C' : ' ';
            e = fmtdesc.flags & 2? 'E' : ' ';
            printf("  %s: %c%c %s\n", fourcc, c, e, fmtdesc.description);
            fmtdesc.index++;
    }

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt)) {
        perror("Querying Pixel Format\n");
        return;
    }
    strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
    printf( "Selected Camera Mode:\n"
            "  Width: %d\n"
            "  Height: %d\n"
            "  PixFmt: %s\n"
            "  Field: %d\n",
            fmt.fmt.pix.width,
            fmt.fmt.pix.height,
            fourcc,
            fmt.fmt.pix.field);

    struct v4l2_streamparm streamparm = {0};
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == ioctl(fd, VIDIOC_G_PARM, &streamparm)) {
        perror("Querying Frame Rate\n");
        return;
    }
    printf( "Frame Rate:  %f\n====================\n",
            (float)streamparm.parm.capture.timeperframe.denominator /
            (float)streamparm.parm.capture.timeperframe.numerator);
}

/* setting the capture param
 *
 *
*/
bool CaptureVideo::setExposureTime(bool in_exp_mode, int in_t)
{    if (in_exp_mode){          // auto exposure
        struct v4l2_control control_s;
        control_s.id = V4L2_CID_EXPOSURE_AUTO;
        control_s.value = V4L2_EXPOSURE_AUTO;
        if( xioctl(fd, VIDIOC_S_CTRL, &control_s) < 0){
            printf("Set Auto Exposure error\n");
            return false;
        }
    }
    else {
        struct v4l2_control control_s;
        control_s.id = V4L2_CID_EXPOSURE_AUTO;
        control_s.value = V4L2_EXPOSURE_MANUAL;
        if( xioctl(fd, VIDIOC_S_CTRL, &control_s) < 0){
            printf("Close MANUAL Exposure error\n");
            return false;
        }

        control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
        control_s.value = in_t;
        if( xioctl(fd, VIDIOC_S_CTRL, &control_s) < 0){
            printf("Set Exposure Time error\n");
            return false;
        }
    }
    return true;
}

bool CaptureVideo::setVideoFormat(int in_width, int in_height, bool in_mjpg)
{
    if (capture_width == in_width && capture_height == in_height)
        return true;
    capture_width = in_width;
    capture_height = in_height;
    current_frame = 0;
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = in_width;
    fmt.fmt.pix.height = in_height;
    if (in_mjpg == true)
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    else
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)){
        printf("Setting Pixel Format\n");
        return false;
    }
    return true;
}

bool CaptureVideo::setVideoFPS(int in_fps)
{
    struct v4l2_streamparm stream_param = {0};
    stream_param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    stream_param.parm.capture.timeperframe.denominator = in_fps;
    stream_param.parm.capture.timeperframe.numerator = 1;

    if (-1 == xioctl(fd, VIDIOC_S_PARM, &stream_param)){
        printf("Setting Frame Rate\n");
        return false;
    }
    return true;
}

bool CaptureVideo::setBufferSize(int in_buffer_size)
{
    if (buffer_size != in_buffer_size){
        buffer_size = in_buffer_size;
        delete [] mb;
        mb = new MapBuffer[buffer_size];
    }
}

/* restart the capture param
 *
 *
*/

bool CaptureVideo::changeVideoFormat(int in_width, int in_height, bool in_mjpg)
{
    closeStream();
    restartCapture();
    setVideoFormat(in_width, in_height, in_mjpg);
    startStream();
    return true;
}

void CaptureVideo::restartCapture()
{
    close(fd);
    fd = open(video_path, O_RDWR);
    buffer_index = 0;
    current_frame = 0;
}
/* get the capture param
 *
 *
*/
bool CaptureVideo::getVideoSize(int &width, int &height)
{
    if (capture_width == 0 || capture_height == 0){
        if (refreshVideoFormat() == false)
            return false;
    }
    width = capture_width;
    height = capture_height;
    return true;
}

CaptureVideo & CaptureVideo::operator>>(Mat &image)
{
    //    std::cout << "current buffr idx: " << buffr_idx << std::endl;
    struct v4l2_buffer bufferinfo = {0};
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = buffer_index;
    if(ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0)
    {
        perror("VIDIOC_DQBUF ERROR");
        exit(1);
    }

    //std::cout << "raw data size: " << bufferinfo.bytesused << std::endl;
    cvtRaw2Mat(mb[buffer_index].ptr, image);

    //memset(&bufferinfo, 0, sizeof(bufferinfo));
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = buffer_index;

    //Queue the next one
    if(ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0)
    {
        perror("VIDIOC_DQBUF ERROR");
        exit(1);
    }
    ++buffer_index;
    buffer_index = buffer_index >= buffer_size ? buffer_index - buffer_size:buffer_index;
    ++current_frame;
    return *this;
}

/* private the capture param
 *
 *
*/
void CaptureVideo::cvtRaw2Mat(const void *data, Mat &image)
{
    if (format == V4L2_PIX_FMT_MJPEG){
        cv::Mat src(capture_height, capture_width, CV_8UC3, (void*) data);
        image = cv::imdecode(src, 1);
    }
    else if(format == V4L2_PIX_FMT_YUYV){
        cv::Mat yuyv(capture_height, capture_width, CV_8UC2, (void*) data);
        cv::cvtColor(yuyv, image, CV_YUV2BGR_YUYV);
    }
}

bool CaptureVideo::refreshVideoFormat()
{
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
    {
        perror("Querying Pixel Format\n");
        return false;
    }
    capture_width = fmt.fmt.pix.width;
    capture_height = fmt.fmt.pix.height;
    format = fmt.fmt.pix.pixelformat;
    return true;
}

bool CaptureVideo::initMMap()
{
    struct v4l2_requestbuffers bufrequest = {0};
    bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufrequest.memory = V4L2_MEMORY_MMAP;
    bufrequest.count = buffer_size;

    if(ioctl(fd, VIDIOC_REQBUFS, &bufrequest) < 0){
        perror("VIDIOC_REQBUFS");
        return false;
    }

    for(int i = 0; i < buffer_size; ++i){
        struct v4l2_buffer bufferinfo = {0};
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = i; /* Queueing buffer index 0. */

        // Put the buffer in the incoming queue.
        if(ioctl(fd, VIDIOC_QUERYBUF, &bufferinfo) < 0){
            perror("VIDIOC_QUERYBUF");
            return false;
        }

        mb[i].ptr = mmap(
                    nullptr,
                    bufferinfo.length,
                    PROT_READ | PROT_WRITE,
                    MAP_SHARED,
                    fd,
                    bufferinfo.m.offset);
        mb[i].size = bufferinfo.length;

        if(mb[i].ptr == MAP_FAILED){
            perror("MAP_FAILED");
            return false;
        }
        memset(mb[i].ptr, 0, bufferinfo.length);

        // Put the buffer in the incoming queue.
        memset(&bufferinfo, 0, sizeof(bufferinfo));
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = i;
        if(ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0){
            perror("VIDIOC_QBUF");
            return false;
        }
    }
    return true;
}

int CaptureVideo::xioctl(int in_fd, int in_request, void *arg)
{
    int r;
    do r = ioctl (fd, in_request, arg);
    while (-1 == r && EINTR == errno);
    return r;
}
char* CaptureVideo::video_name(char* format)
{
    struct tm *newtime;
    char tmpbuf[128];
    time_t test;
    time(&test);
    newtime=localtime(&test);
    strftime(tmpbuf, 128, "%c", newtime);
    char str[100];
    char *firstName = tmpbuf;
    char *name = (char *) malloc(strlen(firstName)+ strlen(format));
    sprintf(name,"%s%s",firstName,format);
    return name;




}
