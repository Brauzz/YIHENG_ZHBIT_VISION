#include "serial_port.h"
#include <unistd.h>     // UNIX Standard Definitions
#include <fcntl.h>      // File Control Definitions
#include <errno.h>      // ERROR Number Definitions
#include <termios.h>    // POSIX Terminal Control Definitions
#include "opencv.hpp"

int cccc = 0;
double t1 = 0.0, t2 = 0.0;
SerialPort::SerialPort(){}
SerialPort::SerialPort(char* filename, int buadrate)
{
    file_name_ = filename;
    buadrate_ = buadrate;
    fd = open(file_name_, O_RDWR | O_NOCTTY | O_SYNC);// Read/Write access to serial port
    cout << fd << endl;                                                // No terminal will control the process
    if(fd == -1)
    {
        printf("open_port: Unable to open %s .\n", file_name_);
    }
    else
    {
        fcntl(fd, F_SETFL,0);
        printf("port is open %s .\n", file_name_);
    }
    struct termios port_settings;               // structure to store the port settings in
    if(buadrate_==0)
    {
        cfsetispeed(&port_settings, B115200);       // set baud rates

        cfsetospeed(&port_settings, B115200);
    }
    else if(buadrate_ == 1)
    {
        cfsetispeed(&port_settings, B921600);       // set baud rates
        cfsetospeed(&port_settings, B921600);
    }
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    port_settings.c_iflag &= ~IGNBRK;         // disable break processing
    port_settings.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    port_settings.c_oflag = 0;                // no remapping, no delays
    port_settings.c_cc[VMIN]  = 0;            // read doesn't block
    port_settings.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;
    port_settings.c_cc[VMIN] = 10;           // read doesn't block
    port_settings.c_cc[VTIME] = 5;          // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings);             // apply the settings to the port
}

// pc -> stm32
void SerialPort::send_data(const struct serial_transmit_data &data)
{
    if(data.size != write(fd, data.raw_data, data.size))
    {
        cout << "!!! send data failure !!!" << fd << endl;
        restart_serial();
        cout << "restart fd" << fd << endl;
    }
}

// stm32 -> pc
bool SerialPort::read_data(const struct serial_receive_data *data, int8_t &mode, int8_t &my_car_color, float &bullet_speed, int8_t &cancel_kalman)
{

    tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */
    unsigned char read_buffer[7];   /* Buffer to store the data received              */
    int  bytes_read = 0;    /* Number of bytes read by the read() system call */

    bytes_read = read(fd,&read_buffer,7); /* Read the data                   */
    if(bytes_read == -1)
    {
        cout << "can not read!" << endl;
        success_ = false;
        return 0;
    }
    //        printf("buffer1 = %d\t\buffer1 = %d\t buffer1 = %d\tbuffer1 = %d\t\n", read_buffer[1], read_buffer[2], read_buffer[3], read_buffer[4]);
    if(read_buffer[0] == data->head && read_buffer[6] == data->end)
    {
        cccc++;
        t1 = cv::getTickCount();
        double t = (t1-t2)*1000/cv::getTickFrequency();
        cout << t  << endl;
        mode = int8_t(read_buffer[1]);
        my_car_color = int8_t(read_buffer[2]);
        bullet_speed = float(short((read_buffer[4]<<8) | read_buffer[3]))/100.0f;
        float bullet_speed_tmp = float(short((read_buffer[4]<<8) | read_buffer[3]))/100.0f;
        cancel_kalman = int8_t(read_buffer[5]);
        if(bullet_speed_tmp < 10){
            bullet_speed = last_bullet_speed;
        }
        else
        {
            last_bullet_speed = bullet_speed_tmp;
            bullet_speed = last_bullet_speed;
        }
        success_ = false;
        t2 = cv::getTickCount();
        return 1;
    }
    cout << "count "<< cccc << endl;
    success_ = true;
    return 0;
}

// gimbal_serial -> pc
bool SerialPort::read_gimbal(const struct serial_gimbal_data* data, float &gimbal_yaw)
{
    //int buffer_size;
    gimbal_yaw = 0;
    /* Discards old data in the rx buffer            */
    unsigned char read_buffer[20];   /* Buffer to store the data received              */
    int  bytes_read = 0;    /* Number of bytes read by the read() system call */

    bytes_read = read(fd,&read_buffer,20); /* Read the data                   */
//        for(int i=0;i<bytes_read;i++)	 /*printing only the received characters*/
//            printf("%x\t",read_buffer[i]);
    if(bytes_read == -1)
    {
        //        cout << "can not read!" << endl;
        return 0;
    }
    //    printf("gim_ax = %d\t\gim_ay = %d\n\n", read_buffer[0], read_buffer[1]);
    for(int i = 0; i < bytes_read; i++)
    {
        if(read_buffer[i] == 0x55 && read_buffer[i+1] == 0x53)
        {
//            float roll = float(short((read_buffer[i+3]<<8) | read_buffer[i+2]))/32768*180;
//            float pitch = float(short((read_buffer[i+5]<<8) | read_buffer[i+4]))/32768*180;
            float yaw = float(short((read_buffer[i+7]<<8) | read_buffer[i+6]))*180/32768;
            //                        printf("gim_pit_angle = %f\t\gim_yaw_angle = %f\n\n", pitch, yaw);
            tcflush(fd, TCIFLUSH);
            gimbal_yaw = yaw;
            success_ = true;
            return 1;
        }
    }
    //    printf("gim_yaw_speed = %f\t\gim_yaw_angle = %f\n\n", gim_pit, gimbal_yaw);
    success_ = false;
    return 0;
}

void SerialPort::restart_serial(void)
{
    close(fd);
    fd = open(file_name_, O_RDWR | O_NOCTTY | O_SYNC);// Read/Write access to serial port
    cout << fd << endl;                                                // No terminal will control the process
    if(fd == -1)
    {
        printf("open_port: Unable to open %s .\n", file_name_);
    }
    else
    {
        fcntl(fd, F_SETFL,0);
        printf("port is open.\n");
    }
    struct termios port_settings;               // structure to store the port settings in
    if(buadrate_==0)
    {
        cfsetispeed(&port_settings, B115200);       // set baud rates

        cfsetospeed(&port_settings, B115200);
    }
    else if(buadrate_ == 1)
    {
        cfsetispeed(&port_settings, B921600);       // set baud rates
        cfsetospeed(&port_settings, B921600);
    }
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    port_settings.c_iflag &= ~IGNBRK;         // disable break processing
    port_settings.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    port_settings.c_oflag = 0;                // no remapping, no delays
    port_settings.c_cc[VMIN]  = 0;            // read doesn't block
    port_settings.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;
    port_settings.c_cc[VMIN] = 10;           // read doesn't block
    port_settings.c_cc[VTIME] = 5;          // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings);             // apply the settings to the port
}

void serial_transmit_data::get_xy_data(int16_t x, int16_t y, int8_t found)
{
    size = 7;
    raw_data[0] = head;
    raw_data[size-1] = end;
    raw_data[1] = y & 0xff;
    raw_data[2] = (y>>8) &0xff;
    raw_data[3] = x & 0xff;
    raw_data[4] = (x>>8) &0xff;
    raw_data[5] = found;
}


