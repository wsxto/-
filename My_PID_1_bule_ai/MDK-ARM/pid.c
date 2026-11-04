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
	float derivative;
	float proportional;
	float differential;
	float unclamped_out;
	float requested_out;
	float previous_out = pid->out;

	pid->target = target;
	pid->actual = actual;
	
	//误差
	pid->error = pid->target - pid->actual;
	
	// Calculate the unconstrained PID output first.
	derivative = pid->error - pid->last_error;
	pid->integral += pid->error;
	proportional = pid->kp * pid->error;
	differential = pid->kd * derivative;
	unclamped_out = proportional
	              + (pid->ki * pid->integral)
	              + differential;
	requested_out = unclamped_out;

	//输出限幅
	if (requested_out > PID_MAX_OUT){
		requested_out = PID_MAX_OUT;
	}
	else if (requested_out < PID_MIN_OUT){
		requested_out = PID_MIN_OUT;
	}

	// Limit the duty-cycle change on every 10 ms control interrupt.
	if (requested_out > previous_out + PID_MAX_RISE_STEP) {
		pid->out = previous_out + PID_MAX_RISE_STEP;
	}
	else if (requested_out < previous_out - PID_MAX_FALL_STEP) {
		pid->out = previous_out - PID_MAX_FALL_STEP;
	}
	else {
		pid->out = requested_out;
	}
	if (pid->out > PID_MAX_OUT) {
		pid->out = PID_MAX_OUT;
	}
	else if (pid->out < PID_MIN_OUT) {
		pid->out = PID_MIN_OUT;
	}

	// Back-calculate the integral so it tracks the PWM actually applied.
	if ((pid->ki != 0.0f) && (pid->out != unclamped_out)) {
		pid->integral = (pid->out - proportional - differential) / pid->ki;
	}
	
	//更新误差
	pid->last_error = pid->error;
	return pid->out;
}


