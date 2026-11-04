#include "My.h"


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		TIM_1ms();
	}

}

void TIM_1ms()
{
	
}


