#include "imu.h"
#include "spi.h"

#define BMI088_GYRO_CHIP_ID_REG  0x00U
#define BMI088_GYRO_CHIP_ID      0x0FU
#define BMI088_GYRO_X_LSB_REG    0x02U
#define BMI088_SPI_READ          0x80U
#define BMI088_SPI_TIMEOUT_MS    10U

static HAL_StatusTypeDef BMI088_Gyro_ReadRegisters(uint8_t reg,
                                                    uint8_t *data,
                                                    uint16_t length)
{
  uint8_t address = reg | BMI088_SPI_READ;
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port,
                    BMI088_GYRO_CS_Pin,
                    GPIO_PIN_RESET);

  status = HAL_SPI_Transmit(&hspi2, &address, 1U, BMI088_SPI_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(&hspi2, data, length, BMI088_SPI_TIMEOUT_MS);
  }

  HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port,
                    BMI088_GYRO_CS_Pin,
                    GPIO_PIN_SET);
  return status;
}

HAL_StatusTypeDef BMI088_Gyro_Init(void)
{
  uint8_t chip_id = 0U;
  HAL_StatusTypeDef status;

  HAL_Delay(10U);
  status = BMI088_Gyro_ReadRegisters(BMI088_GYRO_CHIP_ID_REG, &chip_id, 1U);

  if ((status != HAL_OK) || (chip_id != BMI088_GYRO_CHIP_ID))
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef BMI088_Gyro_ReadRaw(uint8_t raw[BMI088_GYRO_RAW_SIZE])
{
  if (raw == NULL)
  {
    return HAL_ERROR;
  }

  return BMI088_Gyro_ReadRegisters(BMI088_GYRO_X_LSB_REG,
                                    raw,
                                    BMI088_GYRO_RAW_SIZE);
}
