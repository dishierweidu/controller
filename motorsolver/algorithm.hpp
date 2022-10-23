#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <matplotlibcpp.h>

//PID参数
typedef struct _pid_struct
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  
  float ref;      // target value
  float fdb;      // feedback value
  float err[2];   // error and last error

  float p_out;
  float i_out;
  float d_out;
  float output;
}pid_struct;  

//LADRC参数
typedef struct
{   
	float time_cons;//时间常数
	float wo;//观测器带宽
	float b0;//输出增益
	float z1;
	float z2;
    float wc;//控制器带宽
    float max_out;//最大输出
	
	//输入
    float set;//设定值
    float fdb;//反馈值
	float gyro;//角速度
	float err;
	float u;
}ladrc_type_def;

//LADRC参数
typedef struct
{   
	float time_cons;//时间常数
	
	float wo;//观测器带宽
	float b0;//输出增益
	float z1;
	float z2;
    float wc;//控制器带宽
    float K;//控制相关参数
	float s;	
    float max_out;//最大输出
	
	//输入
    float set;//设定值
    float fdb;//反馈值
	float gyro;//角速度
	float smcout;//控制器输出
	float u;	
}ladrsmc_type_def;

//Ladrc_feedforward参数
typedef struct
{   
	float y[2];
	float u[2];
}differ_type_def;

typedef struct
{   
	float time_cons;//时间常数
	
	float wo;//观测器带宽
	float b0;//输出增益
	float z1;
	float z2;
    float wc;//控制器带宽
    float max_out;//最大输出
	float dif1;
	float dif2; 
	float w;//前馈带宽
	float gain;//前馈增益
	differ_type_def differ1;
	differ_type_def differ2;
	
	//输入
    float set;//设定值
	float set_last;//上一次设定值
    float fdb;//反馈值
	float gyro;//角速度
	float err;
	float u;	
}ladrc_fdw_type_def;