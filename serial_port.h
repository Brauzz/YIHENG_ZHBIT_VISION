#pragma once
#include <iostream>
using namespace std;

struct serial_transmit_data
{
    u_char raw_data[10];
    int size;
    u_char head = 0xaa;
    u_char end = 0xbb;
    void get_xy_data(int16_t x, int16_t y, int8_t found);
};

struct serial_receive_data
{
    u_char raw_data[10];
    int size;
    u_char head = 0xaa;
    u_char end = 0xbb;

};

struct serial_gimbal_data
{
    u_char raw_data[20];
    int size;
    u_char head = 0x55;
    u_char end = 0x53;
};

class SerialPort
{
public:
    //"/dev/ttyTHS0"
    SerialPort();
    SerialPort(char* stm32_name, char* gimbal_name);
    SerialPort(char* filename, int buadrate);

    void restart_serial(void);
    void send_data(const struct serial_transmit_data& data);
    bool read_data(const struct serial_receive_data *data, int8_t &mode, int8_t &my_car_color, float &bullet_speed, int8_t& cancel_kalman);
    bool read_gimbal(const struct serial_gimbal_data* data, float &gimbal_yaw);
    int fd;
    bool success_;
private:
    char* file_name_;
    int buadrate_;
    float last_bullet_speed;
};

