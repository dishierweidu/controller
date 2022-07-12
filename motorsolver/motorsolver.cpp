#include "motorsolver.hpp"

/**
 * @brief Construct a new Motor Solver:: Motor Solver object
 * 
 */
MotorSolver::MotorSolver()
{  

}

/**
 * @brief Destroy the Motor Solver:: Motor Solver object
 * 
 */
MotorSolver::~MotorSolver()
{

}

/**
 * @brief 加载MotorSolver参数
 * 
 * @param motor_config_path 参数文件路径
 * @param param_name 参数组名称
 * @return bool 加载成功返回true
 */
bool MotorSolver::loadParam(string motor_config_path,string param_name)
{
    YAML::Node config = YAML::LoadFile(motor_config_path);

    //初始化PID参数
    motor_pid[0].kp = config[param_name]["KP"].as<float>();
    motor_pid[0].ki = config[param_name]["KI"].as<float>();
    motor_pid[0].kd = config[param_name]["KD"].as<float>();

    // cout<<motor_pid[0].kp<<endl;

	pid_init(&motor_pid[0], motor_pid[0].kp,motor_pid[0].ki, motor_pid[0].kd, 30000, 30000,0); //init pid parameter, kp=40, ki=3, kd=0, output limit = 30000


    return true;
}

/**
  * @brief  init PID parameter | 初始化PID参数
  * @param  pid struct
    @param  parameter
	@param	i_max 积分控制器输出上限
	@param	out_max PID控制器输出上限
  * @retval None
  */
void MotorSolver::pid_init(pid_struct *pid,
                                float kp,
                                float ki,
                                float kd,
                                float i_max,
                                float out_max,
                                float i_out)
{
    pid->kp      = kp;
    pid->ki      = ki;
    pid->kd      = kd;
    pid->i_max   = i_max;
    pid->out_max = out_max;
    pid->i_out   = i_out;
}

/**
  * @brief  PID calculation | PID结算
  * @param  pid struct
    @param  reference value
    @param  feedback value
  * @retval calculation result
  */
float MotorSolver::pid_calc(pid_struct *pid, float ref, float fdb)
{
	// std::cout<<"ref : "<<ref<<std::endl;
	// std::cout<<"fdb : "<<fdb<<std::endl;
	pid->ref = ref;
	pid->fdb = fdb;
	pid->err[1] = pid->err[0];
	pid->err[0] = pid->ref - pid->fdb;
	// std::cout<<"p_err[1] : "<<pid->err[1]<<std::endl;
	// std::cout<<"p_err[0] : "<<pid->err[0]<<std::endl;

	pid->p_out  = pid->kp * pid->err[0];
	pid->i_out += pid->ki * pid->err[0];
	pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
	std::cout<<"p_out : "<<pid->p_out<<std::endl;
	std::cout<<"i_out : "<<pid->i_out<<std::endl;
	std::cout<<"d_out : "<<pid->d_out<<std::endl;

	LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);

	pid->output = pid->p_out + pid->i_out + pid->d_out;
	LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);

	std::cout<<"pid->output : "<<pid->output<<std::endl;
	
	return pid->output;
}

float MotorSolver::pid_solver(Motor_Info motor_info[],int target_speed)
{
	// std::cout<<motor_info[0].rotor_angle<<std::endl;
	return pid_calc(&motor_pid[0], target_speed, motor_info[0].rotor_speed);
}