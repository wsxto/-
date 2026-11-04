#ifndef __IMU_H
#define __IMU_H

#include "main.h"

#define CSB2_GPIO GPIOC
#define CSB2_GPIO_PIN GPIO_PIN_3
extern SPI_HandleTypeDef hspi2;
extern volatile float gyro_x;
extern volatile float gyro_y;
extern volatile float gyro_z;
extern volatile int16_t gyro_raw_x;
extern volatile int16_t gyro_raw_y;
extern volatile int16_t gyro_raw_z;
extern volatile uint8_t bmi088_chip_id;
extern volatile uint8_t bmi088_status;
extern volatile uint8_t bmi088_init_ok;
extern volatile uint8_t bmi088_error;
extern volatile HAL_StatusTypeDef bmi088_spi_status;
extern volatile uint32_t bmi088_update_count;

void BMI088_Gyro_WriteReg(uint8_t reg, uint8_t data);
uint8_t BMI088_Gyro_ReadReg(uint8_t reg);
void BIM088_Gyro_ReadData(uint8_t *buf);
void BMI088_Gyro_Init(void);
void BMI088_Gyro_Update(void) ;

#endif

