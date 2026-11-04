/**
 * @file tsk_config_and_callback.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-01-17 1.1 调试到机器人层
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"

#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "2_Device/BSP/BMI088/bsp_bmi088.h"
#include "2_Device/Plotter/Vofa/dvc_vofa.h"
#include "2_Device/BSP/W25Q64JV/bsp_w25q64jv.h"
#include "2_Device/BSP/WS2812/bsp_ws2812.h"
#include "2_Device/BSP/Buzzer/bsp_buzzer.h"
#include "2_Device/BSP/Power/bsp_power.h"
#include "2_Device/BSP/Key/bsp_key.h"
#include "1_Middleware/Algorithm/Filter/Kalman/alg_filter_kalman.h"
#include "1_Middleware/Algorithm/Matrix/alg_matrix.h"
#include "1_Middleware/Driver/WDG/drv_wdg.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "fdcan.h"
#include "1_Middleware/Driver/CAN/drv_can.h"

/* Private macros ------------------------------------------------------------*/
Class_Motor_DJI_C620 Motor_M3508_1;
static uint16_t Motor_Alive_Count = 0;

//临时变量，排查问题
volatile uint32_t dbg_rx_count = 0;
volatile uint32_t dbg_tx_count = 0;
volatile uint8_t dbg_motor_status = 0;
volatile float dbg_target_torque = 0.0f;
volatile float dbg_now_omega = 0.0f;
volatile float dbg_now_torque = 0.0f;
volatile uint8_t dbg_tx_ret = 0;
volatile uint32_t dbg_can_error = 0;
volatile uint32_t dbg_tx_fail_count = 0;
volatile uint32_t dbg_tx_free = 0;
volatile uint32_t dbg_tec = 0;
volatile uint32_t dbg_rec = 0;
volatile uint32_t dbg_bus_off = 0;
volatile uint32_t dbg_error_passive = 0;
volatile uint32_t dbg_last_error = 0;

//接受反馈
static void CAN1_Rx_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    (void)Buffer;

    if (Header.IdType == FDCAN_STANDARD_ID &&
        Header.Identifier == 0x205)
    {
		dbg_rx_count ++;    //debug加入变量
        Motor_M3508_1.CAN_RxCpltCallback();
    }
}
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 全局初始化完成标志位
bool init_finished = false;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 每3600s调用一次
 *
 */
void Task3600s_Callback()
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

/**
 * @brief 每1s调用一次
 *
 */
void Task1s_Callback()
{

}

/**
 * @brief 每1ms调用一次
 *
 */
void Task1ms_Callback()
{
	Motor_Alive_Count++;
	if (Motor_Alive_Count >= 100)
	{
		Motor_Alive_Count = 0;
		Motor_M3508_1.TIM_100ms_Alive_PeriodElapsedCallback();
	}

	dbg_motor_status = (uint8_t)Motor_M3508_1.Get_Status();
	if (dbg_motor_status != Motor_DJI_Status_ENABLE)
	{
		Motor_M3508_1.Set_Target_Torque(0.0f);
	}
	else
	{
		// 临时诊断目标；排查结束后恢复原控制逻辑。
		Motor_M3508_1.Set_Target_Torque(0.3f);
	}
	dbg_target_torque = Motor_M3508_1.Get_Target_Torque();

	Motor_M3508_1.TIM_Calculate_PeriodElapsedCallback();
	dbg_now_omega = Motor_M3508_1.Get_Now_Omega();
	dbg_now_torque = Motor_M3508_1.Get_Now_Torque();

	// 每 1 ms 只发送一次，并记录入队结果及 FDCAN 实时状态。
	uint8_t tx_result = CAN_Transmit_Data(
		&hfdcan1, 0x1FF, CAN1_0x1ff_Tx_Data, 8);
	dbg_tx_ret = (uint8_t)tx_result;
	dbg_tx_count++;
	if (tx_result != HAL_OK)
	{
		dbg_tx_fail_count++;
	}
	dbg_tx_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
	dbg_can_error = HAL_FDCAN_GetError(&hfdcan1);

	FDCAN_ErrorCountersTypeDef error_counters;
	FDCAN_ProtocolStatusTypeDef protocol_status;
	HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters);
	HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status);
	dbg_tec = error_counters.TxErrorCnt;
	dbg_rec = error_counters.RxErrorCnt;
	dbg_bus_off = protocol_status.BusOff;
	dbg_error_passive = protocol_status.ErrorPassive;
	dbg_last_error = protocol_status.LastErrorCode;

	TIM_1ms_IWDG_PeriodElapsedCallback(); // 看门狗
}

/**
 * @brief 每125us调用一次
 *
 */
void Task125us_Callback()
{

}

/**
 * @brief 每10us调用一次
 *
 */
void Task10us_Callback()
{

}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
	
	
	
    SYS_Timestamp.Init(&htim5);
	
	CAN_Init(&hfdcan1,CAN1_Rx_Callback);
	
	Motor_M3508_1.Init(
		&hfdcan1,
		Motor_DJI_ID_0x205,
		Motor_DJI_Control_Method_TORQUE
	);
	
	Motor_M3508_1.Set_Target_Torque(0.3f);
		
    // 定时器中断初始化
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim8);

    // 标记初始化完成
    init_finished = true;
}

/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{

}

/**
 * @brief GPIO中断回调函数
 *
 * @param GPIO_Pin 中断引脚
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (!init_finished)
    {
        return;
    }
}

/**
 * @brief 定时器中断回调函数
 *
 * @param htim
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (!init_finished)
    {
        return;
    }

    // 选择回调函数
    if (htim->Instance == TIM4)
    {
        Task10us_Callback();
    }
    else if (htim->Instance == TIM5)
    {
        Task3600s_Callback();
    }
    else if (htim->Instance == TIM6)
    {
        Task1s_Callback();
    }
    else if (htim->Instance == TIM7)
    {
        Task1ms_Callback();
    }
    else if (htim->Instance == TIM8)
    {
        Task125us_Callback();
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
