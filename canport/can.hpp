#pragma once

#include "can.h"

using namespace std;

typedef struct
{
    uint16_t can_id;            //电机ID (16进制)
    int rotor_angle;            //机械角度
    int rotor_speed;            //转速
    int torque_current;         //实际转矩电流
    int temp;                   //电机温度
}Motor_Info;


class CanPort
{
    public:
        Motor_Info motor_info[7];
        char can_device;
        int set_voltage;            //设定电压值

        bool initCanPort();                      //CAN设备初始化
        bool receive();
        bool send();

    private:
        bool set_Brate();                       //设置CAN设备波特率
        int fd, rx_bytes, tx_bytes;    
        struct sockaddr_can addr;    
        struct ifreq ifr;    
        struct can_frame rx_data;
        struct can_frame frame[8] = {{0}};

        unsigned char voltage[2];

        unsigned char angle[2];
        unsigned char rotating_speed[2];
        unsigned char current[2];
        unsigned char temperature;

        int data_merge(unsigned char *rx_data);
        bool data_split(unsigned char *tx_data,int set_num);
};