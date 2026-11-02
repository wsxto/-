/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float Integral;
    float Last_Error;
} PID_TypeDef;
/**
 * @brief 位置�? PID 计算函数
 * @param pid: 结构体指�?
 * @param target: 目标�? (比如目标温度 40�?)
 * @param measure: 当前测量�? (比如当前温度 30�?)
 * @return: 计算出的控制�? (比如 PWM 占空�?)
 */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile float rpm = 0.0f;
volatile float target_rpm = 100.0f;

PID_TypeDef speed_pid;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PID_DT                0.05f
#define PWM_MAX               100.0f
#define PWM_START_MIN          30.0f
#define RPM_STARTED_THRESHOLD   5.0f
#define ENCODER_CPR          1560.0f

float PID_Calculate(PID_TypeDef *pid,
                    float target,
                    float actual)
{
    float error;
    float derivative;
    float integral_candidate;
    float output;

    /* 1. 计算当前误差 */
    error = target - actual;

    /* 2. 微分：计算误差变化鿟�? */
    derivative = (error - pid->Last_Error) / PID_DT;

    /*
     * 3. 仅在未饱和或积分能使输出逿出饱和时更新积分＿
     * 避免高目标转速时积分持续堆积，随后出现冲高和断续停转�?
     */
    integral_candidate = pid->Integral + pid->Ki * error * PID_DT;
    output = pid->Kp * error
           + integral_candidate
           + pid->Kd * derivative;

    if (((output >= 0.0f) && (output <= PWM_MAX))
        || ((output > PWM_MAX) && (error < 0.0f))
        || ((output < 0.0f) && (error > 0.0f)))
    {
        pid->Integral = integral_candidate;
    }

    if (pid->Integral > PWM_MAX)
    {
        pid->Integral = PWM_MAX;
    }
    else if (pid->Integral < 0.0f)
    {
        pid->Integral = 0.0f;
    }

    /* 4. 使用朿终积分忼计�? PID 输出 */
    output = pid->Kp * error
           + pid->Integral
           + pid->Kd * derivative;

    /* 保存本次误差，下丿次计算微分时使�? */
    pid->Last_Error = error;

    /* 5. PWM 限幅 */
    if (output > PWM_MAX)
    {
        output = PWM_MAX;
    }

    if (output < 0.0f)
    {
        output = 0.0f;
    }

    return output;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
/* PID 初始参数 */
speed_pid.Kp = 0.5f;
speed_pid.Ki = 0.20f;
speed_pid.Kd = 0.00f;

speed_pid.Integral = 0.0f;
speed_pid.Last_Error = 0.0f;

/* 启动编码�? */
HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
__HAL_TIM_SET_COUNTER(&htim1, 0);

/* 承有控制变量初始化后，再启势 50 ms 控制周期 */
HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  if (count >60000)
//    {
//      count = 0;
//      __HAL_TIM_SET_COUNTER(&htim1, 0);
//    }
//    else if (count > 20)
//    {
//      count = 20;
//      __HAL_TIM_SET_COUNTER(&htim1, 20);
//    }
//	  
//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, count*5);


//  
//  HAL_Delay(3000);

//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
//  HAL_Delay(2000);

//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 50);
//  HAL_Delay(3000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    int16_t encoder_count;
    float pwm;

    if (htim->Instance != TIM3)
    {
        return;
    }

    /* Count encoder edges during this 50 ms control period. */
    encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    rpm = (float)encoder_count * 60.0f
        / (ENCODER_CPR * PID_DT);

    /* 当前驱动只有正向 PWM，转速反馈使用大小，避免编码器相序反向㿿 */
    if (rpm < 0.0f)
    {
        rpm = -rpm;
    }

    if (target_rpm <= 0.0f)
    {
        speed_pid.Integral = 0.0f;
        speed_pid.Last_Error = 0.0f;
        pwm = 0.0f;
    }
    else
    {
        pwm = PID_Calculate(&speed_pid, target_rpm, rpm);

        /* 启动时克服静摩擦；转动后允许低于启动占空比以稳定低转速㿿 */
        if ((rpm < RPM_STARTED_THRESHOLD) && (pwm < PWM_START_MIN))
        {
            pwm = PWM_START_MIN;
        }
    }

    __HAL_TIM_SET_COMPARE(&htim2,
                          TIM_CHANNEL_1,
                          (uint32_t)(pwm + 0.5f));
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
