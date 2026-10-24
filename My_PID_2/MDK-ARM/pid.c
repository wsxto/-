#include "pid.h"

PID_TypeDef motor_pid;

void PID_Init(PID_TypeDef *pid,float kp,float ki,float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->target = 0;
	pid->actual = 0;
	pid->error = 0;
	pid->last_error = 0;
	pid->integral = 0;
	pid->out = 0;
}

float PID_Calc(PID_TypeDef*pid,float target,float actual)
{
	pid->target = target;
	pid->actual = actual;
	
	//误差
	pid->error = pid->target - pid->actual;
	
	//积分累加
	pid->integral += pid->error;
	//PID公式计算（位置式）
	pid->out = (pid->kp * pid->error )+(pid->ki * pid->integral)+(pid->kd * (pid->error - pid->last_error));
	//输出限幅

	
	//更新误差
	pid->last_error = pid->error;
	return pid->out;
}


