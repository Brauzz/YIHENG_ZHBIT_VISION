#include <opencv2/opencv.hpp>
#include "image_produce_process.h"
#include <thread>
#include <unistd.h>
using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    ImageProduceProcess ImageControl;
    std::thread produce_task(&ImageProduceProcess::ImageProduce, ImageControl);
    std::thread process_task(&ImageProduceProcess::ImageProcess, ImageControl);
    std::thread gimbal_task(&ImageProduceProcess::GetGimbal, ImageControl);
    std::thread stm32_task(&ImageProduceProcess::GetSTM32, ImageControl);
    produce_task.join();
    process_task.join();
//    gimbal_task.detach();
//    stm32_task.detach();
    return 1;
}

