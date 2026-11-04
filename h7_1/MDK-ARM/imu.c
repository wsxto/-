#include "imu.h"

#define BMI088_GYRO_CHIP_ID_REG       0x00U
#define BMI088_GYRO_CHIP_ID_VALUE     0x0FU
#define BMI088_GYRO_RATE_X_LSB_REG    0x02U
#define BMI088_GYRO_STATUS_REG        0x0AU
#define BMI088_GYRO_RANGE_REG         0x0FU
#define BMI088_GYRO_BANDWIDTH_REG     0x10U
#define BMI088_GYRO_LPM1_REG          0x11U
#define BMI088_GYRO_SOFTRESET_REG     0x14U

#define BMI088_ERROR_NONE             0U
#define BMI088_ERROR_SPI              1U
#define BMI088_ERROR_CHIP_ID          2U
#define BMI088_ERROR_CONFIG           3U

volatile float gyro_x = 0.0f;
volatile float gyro_y = 0.0f;
volatile float gyro_z = 0.0f;
volatile int16_t gyro_raw_x = 0;
volatile int16_t gyro_raw_y = 0;
volatile int16_t gyro_raw_z = 0;
volatile uint8_t bmi088_chip_id = 0U;
volatile uint8_t bmi088_status = 0U;
volatile uint8_t bmi088_init_ok = 0U;
volatile uint8_t bmi088_error = BMI088_ERROR_NONE;
volatile HAL_StatusTypeDef bmi088_spi_status = HAL_OK;
volatile uint32_t bmi088_update_count = 0U;

static void BMI088_Gyro_Select(void)
{
    HAL_GPIO_WritePin(CSB2_GPIO, CSB2_GPIO_PIN, GPIO_PIN_RESET);
}

static void BMI088_Gyro_Deselect(void)
{
    HAL_GPIO_WritePin(CSB2_GPIO, CSB2_GPIO_PIN, GPIO_PIN_SET);
}

void BMI088_Gyro_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2] = {reg & 0x7FU, data};

    BMI088_Gyro_Select();
    bmi088_spi_status = HAL_SPI_Transmit(&hspi2, tx_buf, 2U, 100U);
    BMI088_Gyro_Deselect();
    HAL_Delay(1U);
}

uint8_t BMI088_Gyro_ReadReg(uint8_t reg)
{
    uint8_t tx_buf[2] = {reg | 0x80U, 0x00U};
    uint8_t rx_buf[2] = {0U, 0U};

    BMI088_Gyro_Select();
    bmi088_spi_status = HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 2U, 100U);
    BMI088_Gyro_Deselect();

    return rx_buf[1];
}

void BIM088_Gyro_ReadData(uint8_t *buf)
{
    uint8_t tx_buf[7] = {BMI088_GYRO_RATE_X_LSB_REG | 0x80U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t rx_buf[7] = {0U};
    uint32_t i;

    BMI088_Gyro_Select();
    bmi088_spi_status = HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 7U, 100U);
    BMI088_Gyro_Deselect();

    for (i = 0U; i < 6U; i++) {
        buf[i] = rx_buf[i + 1U];
    }
}

void BMI088_Gyro_Init(void)
{
    bmi088_init_ok = 0U;
    bmi088_error = BMI088_ERROR_NONE;
    BMI088_Gyro_Deselect();
    HAL_Delay(30U);

    /* Reset the gyro so MCU-only resets cannot leave stale sensor state. */
    BMI088_Gyro_WriteReg(BMI088_GYRO_SOFTRESET_REG, 0xB6U);
    HAL_Delay(30U);

    bmi088_chip_id = BMI088_Gyro_ReadReg(BMI088_GYRO_CHIP_ID_REG);
    if (bmi088_spi_status != HAL_OK) {
        bmi088_error = BMI088_ERROR_SPI;
        return;
    }
    if (bmi088_chip_id != BMI088_GYRO_CHIP_ID_VALUE) {
        bmi088_error = BMI088_ERROR_CHIP_ID;
        return;
    }

    /* Normal mode, +/-2000 dps, 2000 Hz ODR and 532 Hz bandwidth. */
    BMI088_Gyro_WriteReg(BMI088_GYRO_LPM1_REG, 0x00U);
    BMI088_Gyro_WriteReg(BMI088_GYRO_RANGE_REG, 0x00U);
    BMI088_Gyro_WriteReg(BMI088_GYRO_BANDWIDTH_REG, 0x80U);
    HAL_Delay(30U);

    if ((BMI088_Gyro_ReadReg(BMI088_GYRO_RANGE_REG) != 0x00U) ||
        (BMI088_Gyro_ReadReg(BMI088_GYRO_BANDWIDTH_REG) != 0x80U)) {
        bmi088_error = BMI088_ERROR_CONFIG;
        return;
    }

    bmi088_init_ok = 1U;
}

void BMI088_Gyro_Update(void)
{
    uint8_t raw_data[6];

    if (bmi088_init_ok == 0U) {
        return;
    }

    bmi088_status = BMI088_Gyro_ReadReg(BMI088_GYRO_STATUS_REG);
    if (bmi088_spi_status != HAL_OK) {
        bmi088_error = BMI088_ERROR_SPI;
        bmi088_init_ok = 0U;
        return;
    }

    /* Gyro output registers can be burst-read continuously in normal mode. */
    BIM088_Gyro_ReadData(raw_data);
    if (bmi088_spi_status != HAL_OK) {
        bmi088_error = BMI088_ERROR_SPI;
        bmi088_init_ok = 0U;
        return;
    }

    gyro_raw_x = (int16_t)(((uint16_t)raw_data[1] << 8U) | raw_data[0]);
    gyro_raw_y = (int16_t)(((uint16_t)raw_data[3] << 8U) | raw_data[2]);
    gyro_raw_z = (int16_t)(((uint16_t)raw_data[5] << 8U) | raw_data[4]);

    gyro_x = (float)gyro_raw_x / 16.4f;
    gyro_y = (float)gyro_raw_y / 16.4f;
    gyro_z = (float)gyro_raw_z / 16.4f;
    bmi088_update_count++;
}
