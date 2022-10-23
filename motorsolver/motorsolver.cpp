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
  * @param  parameter
	* @param	i_max 积分控制器输出上限
	* @param	out_max PID控制器输出上限
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
  * @brief  PID calculation | PID解算
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

/**
 * @brief 限幅函数
 * 
 * @param x 输入
 * @param min 最小值
 * @param max 最大值
 */
void MotorSolver::ladrc_init(ladrc_type_def *ladrc,
                        float wc,
                        float b0,
                        float wo,
                        float max_out)
{
  ladrc->wc = wc;
  ladrc->b0 = b0;
  ladrc->wo = wo;
  ladrc->max_out = max_out;

  ladrc->fdb = 0.0f;
  ladrc->u = 0.0f;
  ladrc->set = 0.0f;
  ladrc->gyro = 0.0f;
  ladrc->z1 = 0;
  ladrc->z2 = 0;
  ladrc->time_cons = 0.002;//采样率
}

/**
 * @brief Ladrc calculation | Ladrc解算
 * 
 * 
 */
float MotorSolver::ladrc_calc(ladrc_type_def *ladrc, float ref, float set, float gyro)
{
  float err;
	err = set - ref;
	
	
	ladrc->set = set;
  ladrc->fdb = ref;
	ladrc->err = rad_format(err);
	ladrc->gyro = gyro;
	
	//零阶保持法离散化积分器
	ladrc->z2 += ladrc->time_cons*(ladrc->wo*ladrc->wo)*(ladrc->gyro-ladrc->z1);
	ladrc->z1 += ladrc->time_cons*((ladrc->b0*ladrc->u) + ladrc->z2 + (2*ladrc->wo)*(ladrc->gyro-ladrc->z1));
	ladrc->u = (ladrc->wc*ladrc->wc*ladrc->err - 2*ladrc->wc*ladrc->z1 - ladrc->z2)/ladrc->b0;
	LimitMax(ladrc->u, ladrc->max_out);
   
	return ladrc->u;
}

/**
 * @brief Ladrsmc calculation | Ladrsmc解算
 * 
 * @param x 输入
 * @param min 最小值
 * @param max 最大值
 * @return float 限幅后的值
 */
float MotorSolver::ladrsmc_calc(ladrsmc_type_def *ladrsmc, float ref, float fdb)
{
  return 0;
}

/**
 * @brief Ladrc_feedforward calculation | Ladrc_feedforward解算
 * 
 * @param x 输入
 * @param min 最小值
 * @param max 最大值
 * @return float 限幅后的值
 */
float MotorSolver::ladrc_fdw_calc(ladrc_fdw_type_def *ladrc_fdw_calc, float ref, float fdb)
{
  return 0;
}

float MotorSolver::pid_solver(Motor_Info motor_info[],int target_speed)
{
	// std::cout<<motor_info[0].rotor_angle<<std::endl;
	return pid_calc(&motor_pid[0], target_speed, motor_info[0].rotor_speed);
}
