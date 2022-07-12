#include "can.hpp"

bool CanPort::set_Brate()
{
    return true;
}

bool CanPort::initCanPort()
{   
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字    
    strcpy(ifr.ifr_name, "can0");    
    ioctl(s, SIOCGIFINDEX, &ifr);//指定can0设备    
    addr.can_family = AF_CAN;    
    addr.can_ifindex = ifr.ifr_ifindex;    
    bind(s, (struct sockaddr*)&addr, sizeof(addr));//将套接字与can0绑定  

    //禁用过滤规则，本进程不接收报文，只负责发送    
    // setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);  

    //循环发送两个报文    
    // while(1)    
    // {

    //     // = pid_calc(&motor_pid[0], target_speed, now_rotating_speed);
        

    //     usleep(1000);
    // }    
    // close(s);    
    return true;
}

bool CanPort::receive()
{
    //Receive
    rx_bytes = read(s, &rx_data, sizeof(rx_data));//接收报文        
    //显示报文        
    if((rx_bytes>0) && (rx_data.can_id = '0x205'))        
    {            
        // printf("ID=0x%XDLC=%ddata[0]=0x%X\n", rx_data.can_id, rx_data.can_dlc, rx_data.data[0]);

        angle[0] = rx_data.data[0];
        angle[1] = rx_data.data[1];
        rotating_speed[0] = rx_data.data[2];
        rotating_speed[1] = rx_data.data[3]; 
        current[0] = rx_data.data[4];
        current[1] = rx_data.data[5];
        temperature = rx_data.data[6];

        motor_info[0].rotor_angle = data_merge(angle);
        motor_info[0].rotor_speed = data_merge(rotating_speed);
        motor_info[0].torque_current = data_merge(current);
        motor_info[0].temp = int(temperature);
        
        printf("now_angle = %d   now_rotating_speed = %d   now_current = %d   now_temperature = %d\n",
        motor_info[0].rotor_angle, motor_info[0].rotor_speed, motor_info[0].torque_current, motor_info[0].temp);
        return true;
    }
    else
        return false;
}

bool CanPort::send()
{
    // std::cout<<"set_voltage : "<<set_voltage<<std::endl; 
    data_split(voltage,set_voltage);

    //生成两个报文    
    frame[0].can_id = 0x1FF;   
    frame[0].can_dlc = 8;
    frame[0].data[0] = voltage[0];
    frame[0].data[1] = voltage[1];

    // Send
    tx_bytes = write(s,&frame[0], sizeof(frame[0]));//发送frame[0]        
    if(tx_bytes != sizeof(frame[0]))        
    {       
        std::cout<<sizeof(frame[0])<<std::endl;
        std::cout<<tx_bytes<<std::endl;   
        printf("Send Error frame[0]\n!");            
        return false; 
    }
    return true;
}

int CanPort::data_merge(unsigned char *rx_data)
{
    return (((*((uint8_t *)rx_data + 1 )<< 8))| *(uint8_t *)rx_data);
}

bool CanPort::data_split(unsigned char *tx_data,int set_num)
{
    tx_data[0] = set_num >> 8;      //高八位
    tx_data[1] = set_num;           //低八位
    return true;
}