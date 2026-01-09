#include "tof_io.h"

#include "main.h"
#include "stm32f7xx_hal_def.h"
#include "stm32f7xx_hal_spi.h"
#include <stdint.h>
#include <string.h>
#include <vl53l8cx_api.h>
#include <stm32f7xx_hal.h>

extern SPI_HandleTypeDef hspi2;

int32_t TOFIO_Init(void)
{
    return 0;
}

int32_t TOFIO_DeInit(void)
{
    return 0;
}

int32_t TOFIO_WriteReg(uint16_t address, uint16_t reg, uint8_t *data, uint16_t length)
{
    (void)address;
    uint8_t tx_buf[2] = {
        reg >> 8U,
        reg & 0xFF
    };
    HAL_GPIO_WritePin(TOF_CS1_GPIO_Port, TOF_CS1_Pin, 0);
    HAL_SPI_Transmit(&hspi2, tx_buf, 2, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi2, data, length, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(TOF_CS1_GPIO_Port, TOF_CS1_Pin, 1);
    return 0;
}

int32_t TOFIO_ReadReg(uint16_t address, uint16_t reg, uint8_t *data, uint16_t length)
{
    // if (length > 6) {
    //     return -1;
    // }
    // (void)address;
    // uint8_t tx_buf[8] = {
    //     reg >> 8U,
    //     reg & 0xFF
    // };
    // uint8_t rx_buf[8];
    // HAL_GPIO_WritePin(TOF_CS1_GPIO_Port, TOF_CS1_Pin, 0);
    // HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, length + 2, HAL_MAX_DELAY);
    // HAL_GPIO_WritePin(TOF_CS1_GPIO_Port, TOF_CS1_Pin, 1);
    // memcpy(data, rx_buf + 2U, length);
    (void)address;
    uint8_t tx_buf[2] = {
        reg >> 8U,
        reg & 0xFF
    };
    HAL_GPIO_WritePin(TOF_CS1_GPIO_Port, TOF_CS1_Pin, 0);
    HAL_SPI_Transmit(&hspi2, tx_buf, 2, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, data, length, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(TOF_CS1_GPIO_Port, TOF_CS1_Pin, 1);
    return 0;
}

int32_t TOFIO_GetTick(void)
{
    return HAL_GetTick();
}