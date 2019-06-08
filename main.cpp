/****************************************************************************
 *  Copyright (C) 2019 cz.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#include <opencv2/opencv.hpp>
#include "thread_control.h"
#include <thread>
#include <unistd.h>
using namespace std;
using namespace cv;

int main()
{
    // 开启相关线程
    ThreadControl ImageControl;
    // 图像生成线程（短焦和长焦）
    std::thread produce_task(&ThreadControl::ImageProduce, ImageControl);
    std::thread produce_long_task(&ThreadControl::ImageProduceLong, ImageControl);
    // 图像处理线程（自瞄、打符、串口）
    std::thread process_task(&ThreadControl::ImageProcess, ImageControl);
    // 串口数据接受线程
//    std::thread gimbal_task(&ImageProduceProcess::GetGimbal, ImageControl);
//    std::thread stm32_task(&ImageProduceProcess::GetSTM32, ImageControl);

    produce_task.join();
    produce_long_task.join();
    process_task.join();
//    gimbal_task.detach();
//    stm32_task.detach();
    return 1;
}

