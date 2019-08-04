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
    std::thread produce_task(&ThreadControl::ImageProduce, &ImageControl);  // & == std::ref()
    // 图像处理线程（自瞄、打符、串口）
    std::thread process_task(&ThreadControl::ImageProcess, &ImageControl);


#ifdef GET_STM32_THREAD
    std::thread stm32_task(&ThreadControl::GetSTM32, &ImageControl);
#endif
#ifdef SAVE_VIDEO_THREAD
    std::thread write_task(&ThreadControl::ImageWrite, &ImageControl);
#endif

    produce_task.join();
    process_task.join();

#ifdef GET_STM32_THREAD
    stm32_task.detach();
#endif
#ifdef SAVE_VIDEO_THREAD
    write_task.join();
#endif
    return 1;
}

