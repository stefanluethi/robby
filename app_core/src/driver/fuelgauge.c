#include "fuelgauge.h"
#include "stm32f7xx_hal.h"

extern I2C_HandleTypeDef hi2c2;

#define BQ28Z620_I2C_ADDR (0x55 << 1)
#define CMD_CURRENT 0x0A
#define CMD_RELATIVE_STATE_OF_CHARGE 0x0D
#define CMD_CELL2_VOLTAGE 0x3E
#define CMD_CELL1_VOLTAGE 0x3F

#define I2C_TIMEOUT 1000

static bool fuelgauge_read_reg16(uint8_t reg, uint16_t *value) {
    uint8_t data[2] = {0, 0};
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, 
                                                BQ28Z620_I2C_ADDR, 
                                                reg, 
                                                I2C_MEMADD_SIZE_8BIT, 
                                                data, 
                                                2, 
                                                I2C_TIMEOUT);
                                                
    if (status == HAL_OK) {
        *value = (uint16_t)(data[0] | (data[1] << 8));
        return true;
    }
    return false;
}

bool fuelgauge_init(void) {
    /* Check if the device is ready and responding on the I2C bus */
    if (HAL_I2C_IsDeviceReady(&hi2c2, BQ28Z620_I2C_ADDR, 2, I2C_TIMEOUT) == HAL_OK) {
        return true;
    }
    return false;
}

bool fuelgauge_get_soc(uint8_t *soc) {
    if (soc == NULL) {
        return false;
    }

    uint16_t value = 0;
    if (fuelgauge_read_reg16(CMD_RELATIVE_STATE_OF_CHARGE, &value)) {
        if (value > 100) {
            value = 100; /* Cap at 100% per SBS specification limits */
        }
        *soc = (uint8_t)value;
        return true;
    }

    return false;
}

bool fuelgauge_get_current(int16_t *current) {
    if (current == NULL) {
        return false;
    }

    uint16_t value = 0;
    if (fuelgauge_read_reg16(CMD_CURRENT, &value)) {
        *current = (int16_t)value; /* Cast to signed integer for current */
        return true;
    }

    return false;
}

bool fuelgauge_get_cell1_voltage(uint16_t *voltage) {
    if (voltage == NULL) {
        return false;
    }

    return fuelgauge_read_reg16(CMD_CELL1_VOLTAGE, voltage);
}

bool fuelgauge_get_cell2_voltage(uint16_t *voltage) {
    if (voltage == NULL) {
        return false;
    }

    return fuelgauge_read_reg16(CMD_CELL2_VOLTAGE, voltage);
}
