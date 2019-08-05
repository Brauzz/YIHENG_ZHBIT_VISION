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
#include "serial_port.h"
#include <unistd.h>     // UNIX Standard Definitions
#include <fcntl.h>      // File Control Definitions
#include <errno.h>      // ERROR Number Definitions
#include <termios.h>    // POSIX Terminal Control Definitions

SerialPort::SerialPort(){}

SerialPort::SerialPort(const char* filename, int buadrate)
{
    file_name_ = filename;
    buadrate_ = buadrate;
    success_ = false;
//    serial_mode = NO_INIT;
    fd = open(file_name_, O_RDWR | O_NOCTTY | O_SYNC);// Read/Write access to serial port                                           // No terminal will control the process
    last_fd = fd;
    if(fd == -1)
    {
        printf("open_port wait to open %s .\n", file_name_);
//        NOTICE("wait serial " << file_name_,1);
        return;
    }
    else if(fd != -1 )
    {
        fcntl(fd, F_SETFL,0);
//        NOTICE("port is open " << file_name_,1);
        printf("port is open %s.\n", file_name_);
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
bool SerialPort::read_data(const struct serial_receive_data *data, bool &mode, bool &my_car_color
                           , int &buff_offset_x, int &buff_offset_y)
{

    tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */
    unsigned char read_buffer[7];   /* Buffer to store the data received              */
    int  bytes_read = 0;    /* Number of bytes read by the read() system call */

    bytes_read = read(fd,&read_buffer,7); /* Read the data                   */
//    cout << "bytes_read: " << bytes_read;
//    for (int i = 0; i < bytes_read; i++) {
//        cout << "buf " << i << ": " << read_buffer[i];
//    }
//    cout << endl;
    if(bytes_read == -1 || bytes_read == 0)
    {
//        cout << "can not read!" << endl;
//        NOTICE("can not read!",3);
        restart_serial();
        success_ = false;
        return 0;
    }
//    printf("buffer1 = %d\t\buffer1 = %d\t buffer1 = %d\tbuffer1 = %d\t\n", read_buffer[1], read_buffer[2], read_buffer[3], read_buffer[4]);
    if(read_buffer[0] == data->head && read_buffer[6] == data->end)
    {
        NOTICE("Get stm32 data sucessed!!!", 3)
        mode = bool(read_buffer[1]);
//        printf("buffer1 = %d\r\n", read_buffer[1]);
        my_car_color = bool(read_buffer[2]);
        buff_offset_x = char(read_buffer[3]);
        buff_offset_y = char(read_buffer[4]);
//        cout << "x: " << buff_offset_x << " y:" << buff_offset_y << endl;
        //        gimbal_data = float(short((read_buffer[4]<<8) | read_buffer[3]))/100.0f;
//        cout << gimbal_data<< endl;
        success_ = true;
        return 1;
    }
//    cout << "count "<< cccc << endl;
    success_ = false;
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
    if(bytes_read == -1 || bytes_read == 0)
    {
        restart_serial();
        success_ = false;
        //        cout << "can not read!" << endl;
        return 0;
    }
//        printf("gim_ax = %d\t\gim_ay = %d\n\n", read_buffer[0], read_buffer[1]);
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
//    cout << "test restart !!" << fd << " " << last_fd << endl;
    close(fd);
    fd = open(file_name_, O_RDWR | O_NOCTTY | O_SYNC);// Read/Write access to serial port
//    cout << serial_mode << endl;
    if(fd == -1 && last_fd != -1)
    {
        printf("open_port wait to open %s .\n", file_name_);
//        NOTICE("wait serial",1);
        last_fd = fd;
        return;
    }
    else if(fd != -1 && last_fd == -1)
    {
        fcntl(fd, F_SETFL,0);
//        NOTICE("port is open",1);
        printf("port is open %s.\n", file_name_);
        last_fd = fd;
    }else
    {
        last_fd = fd;
        return;
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

void serial_transmit_data::get_xy_data(int16_t x, int16_t y, int found)
{
    size = 7;
    raw_data[0] = head;
    raw_data[size-1] = end;
    raw_data[1] = y & 0xff;
    raw_data[2] = (y>>8) &0xff;
    raw_data[3] = x & 0xff;
    raw_data[4] = (x>>8) &0xff;
    raw_data[5] = static_cast<unsigned char>(found);
}


