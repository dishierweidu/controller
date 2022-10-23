#pragma once

#include "algorithm.hpp"
#include "../canport/can.hpp"

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

using namespace std;
namespace plt = matplotlibcpp;

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

        void ladrc_init(ladrc_type_def *ladrc,
                        float wc,
                        float b0,
                        float wo,
                        float max_out);
        float ladrc_calc(ladrc_type_def *ladrc_calc, float ref, float set, float gyro);
        float ladrsmc_calc(ladrsmc_type_def *ladrcsmc_calc, float ref, float fdb);
        float ladrc_fdw_calc(ladrc_fdw_type_def *ladrc_fdw_calc, float ref, float fdb);
};