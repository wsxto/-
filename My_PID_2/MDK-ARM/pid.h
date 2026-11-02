#include "main.h"

#define PID_MAX_OUT 0.f
#define PID_MIN_OUT 0.f

typedef struct {
		float kp;
		float ki;
		float kd;
		
		float target;
		float actual;
		float error;
		float last_error;
		float integral;			//积分累积项
		
		float out;
}PID_TypeDef;

extern PID_TypeDef motor_pid;

void PID_Init(PID_TypeDef*pid,float kp,float ki,float kd);
float PID_Calc(PID_TypeDef*pid,float target,float actual);
