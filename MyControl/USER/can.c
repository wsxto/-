#include "My.h"

void CAN_Init(FDCAN_HandleTypeDef *hfdcan, CAN_Callback Callback_Function)
{
    if (hfdcan->Instance == FDCAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hfdcan;
        CAN1_Manage_Object.Callback_Function = Callback_Function;
    }
 
   
    can_filter_mask_config(hfdcan);

    HAL_FDCAN_Start(hfdcan);
}
FDCAN_FilterTypeDef  filter = {0};





