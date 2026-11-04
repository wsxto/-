#ifndef __IMU_H__
#define __IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define BMI088_GYRO_RAW_SIZE 6U

HAL_StatusTypeDef BMI088_Gyro_Init(void);
HAL_StatusTypeDef BMI088_Gyro_ReadRaw(uint8_t raw[BMI088_GYRO_RAW_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H__ */
