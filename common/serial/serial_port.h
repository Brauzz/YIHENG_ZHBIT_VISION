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
#pragma once
#include <iostream>
#include "../../base.h"
using namespace std;

struct serial_transmit_data
{
    unsigned char raw_data[10];
    int size;
    unsigned char head = 0xaa;
    unsigned char end = 0xbb;
    void get_xy_data(int16_t x, int16_t y, int found);
};

struct serial_receive_data
{
    unsigned char raw_data[10];
    int size;
    unsigned char head = 0xaa;
    unsigned char end = 0xbb;

};

struct serial_gimbal_data
{
    unsigned char raw_data[20];
    int size;
    unsigned char head = 0x55;
    unsigned char end = 0x53;
};

class SerialPort
{
public:
    //"/dev/ttyTHS0"
    SerialPort();
    /**
     * @brief SerialPort
     * @param filename 串口名字
     * @param buadrate 串口波特率,用于stm32通信是0-B115200,用于stm32通信是1-B921600
     */
    SerialPort(const char* filename, int buadrate);

    void restart_serial(void);  // 尝试重连的函数
    void send_data(const struct serial_transmit_data& data);
    bool read_data(const struct serial_receive_data *data, bool &mode, bool &my_car_color
                   , int &buff_offset_x, int &buff_offset_y);
    bool read_gimbal(const struct serial_gimbal_data* data, float &gimbal_yaw);
    int fd;
    int last_fd;
    bool success_;

private:

    const char* file_name_;
    int buadrate_;
    float last_bullet_speed;
};

