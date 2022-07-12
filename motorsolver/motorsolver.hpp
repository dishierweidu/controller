#pragma once

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <matplotlibcpp.h>
#include "../canport/can.hpp"

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

using namespace std;
namespace plt = matplotlibcpp;

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

class MotorSolver
{
    public:
        MotorSolver();
        ~MotorSolver();
        
        char can_device;

        bool loadParam(string coord_path,string param_name);
        float pid_solver(Motor_Info motor_info[],int target_speed);

    private:
        pid_struct motor_pid[7];
        void pid_init(pid_struct *pid,
                        float kp,
                        float ki,
                        float kd,
                        float i_max,
                        float out_max,
                        float i_out);
        float pid_calc(pid_struct *pid, float ref, float fdb);
};