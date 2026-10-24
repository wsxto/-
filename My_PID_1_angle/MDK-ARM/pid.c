#include "pid.h"

PID_TypeDef motor_pid;
PID_TypeDef post_pid;

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

	//计算
	pid->integral += pid->error;
	pid->out = (pid->error * pid->kp) + pid->ki * (pid->integral) + (pid->error - pid->last_error) * pid->kd;

	//输出限幅
	if (pid->out > PID_MAX_OUT){
		pid->out = PID_MAX_OUT;
	}
	else if (pid->out < PID_MIN_OUT){
		pid->out = PID_MIN_OUT;
	}

	if (pid->out > PID_MAX_OUT) {
		pid->out = PID_MAX_OUT;
	}
	else if (pid->out < PID_MIN_OUT) {
		pid->out = PID_MIN_OUT;
	}

	//更新误差
	pid->last_error = pid->error;
	return pid->out;
}

