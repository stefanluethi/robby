#include "fuelgauge.h"
#include "stm32f7xx_hal.h"

extern I2C_HandleTypeDef hi2c2;

#define BQ28Z620_I2C_ADDR (0x55 << 1)
#define CMD_CURRENT 0x0A
#define CMD_RELATIVE_STATE_OF_CHARGE 0x2C
#define CMD_VOLTAGE 0x08
#define MAC_BLOCK_ACCESS 0x44
#define SUBCMD_DASTATUS1 0x0071

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
    /* Check if the device is ready and responding on the I2C bus.
     * BQ28Z620 and similar smart gauges often NACK an address-only ping
     * from HAL_I2C_IsDeviceReady, so we read a register instead.
     */
    uint16_t dummy = 0;
    if (fuelgauge_read_reg16(CMD_VOLTAGE, &dummy)) {
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

static bool fuelgauge_read_dastatus1(uint16_t *cell1, uint16_t *cell2) {
    // SMBus Block Write format requires the Byte Count after the Command.
    // HAL_I2C_Mem_Write sends: [MemAddress] -> [Data...]
    // For command 0x44, Data[0] must be the Byte Count (2), then the 2-byte sub-command.
    uint8_t write_buf[3] = { 0x02, SUBCMD_DASTATUS1 & 0xFF, (SUBCMD_DASTATUS1 >> 8) & 0xFF };
    uint8_t read_buf[7];
    
    if (HAL_I2C_Mem_Write(&hi2c2, BQ28Z620_I2C_ADDR, MAC_BLOCK_ACCESS, I2C_MEMADD_SIZE_8BIT, write_buf, 3, I2C_TIMEOUT) != HAL_OK) {
        return false;
    }
    
    if (HAL_I2C_Mem_Read(&hi2c2, BQ28Z620_I2C_ADDR, MAC_BLOCK_ACCESS, I2C_MEMADD_SIZE_8BIT, read_buf, 7, I2C_TIMEOUT) != HAL_OK) {
        return false;
    }
    
    if (read_buf[1] != (SUBCMD_DASTATUS1 & 0xFF) || read_buf[2] != ((SUBCMD_DASTATUS1 >> 8) & 0xFF)) {
        return false;
    }
    
    if (cell1) {
        *cell1 = (uint16_t)(read_buf[3] | (read_buf[4] << 8));
    }
    if (cell2) {
        *cell2 = (uint16_t)(read_buf[5] | (read_buf[6] << 8));
    }
    return true;
}

bool fuelgauge_get_cell1_voltage(uint16_t *voltage) {
    if (voltage == NULL) {
        return false;
    }

    return fuelgauge_read_dastatus1(voltage, NULL);
}

bool fuelgauge_get_cell2_voltage(uint16_t *voltage) {
    if (voltage == NULL) {
        return false;
    }

    return fuelgauge_read_dastatus1(NULL, voltage);
}
