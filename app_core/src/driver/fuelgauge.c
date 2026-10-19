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

/* MAC access through the AltManufacturerAccess() registers, see TI SLUUCO9B */
#define CMD_ALT_MANUFACTURER_ACCESS 0x3E
#define CMD_MAC_DATA_CHECKSUM 0x60
#define SUBCMD_OPERATION_STATUS 0x0054
#define OPERATION_STATUS_SEC_SHIFT 8
#define OPERATION_STATUS_SEC_MASK 0x3U
#define OPERATION_STATUS_SEC_SEALED 0x3U

/* Settings/Configuration/I2C Configuration, bit 0 enables BROADCAST mode */
#define DF_I2C_CONFIGURATION 0x4602
#define I2C_CONFIGURATION_BCAST 0x01U

/* The gauge only answers in standard mode, its 400 kHz XL mode is off.
 * ~100 kHz from PCLK1 = 54 MHz. */
#define I2C2_TIMING_100KHZ 0xA0421216
#define I2C_ATTEMPTS 3
#define MAC_RESPONSE_DELAY_MS 2
#define DF_WRITE_DELAY_MS 100

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

static void i2c2_set_timing(uint32_t timing) {
    /* TIMINGR is only writable while the peripheral is disabled */
    __HAL_I2C_DISABLE(&hi2c2);
    hi2c2.Instance->TIMINGR = timing;
    __HAL_I2C_ENABLE(&hi2c2);
}

/* The gauge becomes bus master while it broadcasts, so a transfer can lose arbitration */
static HAL_StatusTypeDef fuelgauge_write(uint8_t reg, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = HAL_ERROR;
    for (int i = 0; i < I2C_ATTEMPTS && status != HAL_OK; ++i) {
        status = HAL_I2C_Mem_Write(&hi2c2, BQ28Z620_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, size, I2C_TIMEOUT);
    }
    return status;
}

static HAL_StatusTypeDef fuelgauge_read(uint8_t reg, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = HAL_ERROR;
    for (int i = 0; i < I2C_ATTEMPTS && status != HAL_OK; ++i) {
        status = HAL_I2C_Mem_Read(&hi2c2, BQ28Z620_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, size, I2C_TIMEOUT);
    }
    return status;
}

/* Sends a MAC command or data flash address and reads back its echo followed by the data */
static bool fuelgauge_mac_read(uint16_t command, uint8_t *data, uint16_t size) {
    uint8_t buf[2 + 4];
    if (size > sizeof(buf) - 2) {
        return false;
    }

    buf[0] = command & 0xFF;
    buf[1] = (command >> 8) & 0xFF;
    if (fuelgauge_write(CMD_ALT_MANUFACTURER_ACCESS, buf, 2) != HAL_OK) {
        return false;
    }

    HAL_Delay(MAC_RESPONSE_DELAY_MS);

    if (fuelgauge_read(CMD_ALT_MANUFACTURER_ACCESS, buf, 2 + size) != HAL_OK) {
        return false;
    }
    if (buf[0] != (command & 0xFF) || buf[1] != ((command >> 8) & 0xFF)) {
        return false;
    }

    for (uint16_t i = 0; i < size; ++i) {
        data[i] = buf[2 + i];
    }
    return true;
}

static bool fuelgauge_df_write_byte(uint16_t address, uint8_t value) {
    uint8_t buf[3] = { address & 0xFF, (address >> 8) & 0xFF, value };
    if (fuelgauge_write(CMD_ALT_MANUFACTURER_ACCESS, buf, sizeof(buf)) != HAL_OK) {
        return false;
    }

    /* The write is committed once MACDataChecksum() and MACDataLength() match,
     * the length counts the address, data, checksum and length bytes */
    uint8_t sum = buf[0] + buf[1] + buf[2];
    uint8_t checksum[2] = { (uint8_t)~sum, sizeof(buf) + 2 };
    return fuelgauge_write(CMD_MAC_DATA_CHECKSUM, checksum, sizeof(checksum)) == HAL_OK;
}

static fuelgauge_broadcast_result_t fuelgauge_clear_broadcast(uint8_t *i2c_configuration) {
    uint8_t buf[4];

    /* The very first transfer tells whether a gauge is there at all */
    if (!fuelgauge_mac_read(SUBCMD_OPERATION_STATUS, buf, sizeof(buf))) {
        return (HAL_I2C_GetError(&hi2c2) & HAL_I2C_ERROR_AF) ? FUELGAUGE_BROADCAST_NO_GAUGE
                                                             : FUELGAUGE_BROADCAST_FAILED;
    }

    /* A SEALED gauge doesn't give access to its data flash */
    uint32_t operation_status = buf[0] | (buf[1] << 8) | (buf[2] << 16) | ((uint32_t)buf[3] << 24);
    if (((operation_status >> OPERATION_STATUS_SEC_SHIFT) & OPERATION_STATUS_SEC_MASK) == OPERATION_STATUS_SEC_SEALED) {
        return FUELGAUGE_BROADCAST_SEALED;
    }

    if (!fuelgauge_mac_read(DF_I2C_CONFIGURATION, i2c_configuration, 1)) {
        return FUELGAUGE_BROADCAST_FAILED;
    }
    if ((*i2c_configuration & I2C_CONFIGURATION_BCAST) == 0) {
        return FUELGAUGE_BROADCAST_OFF;
    }

    if (!fuelgauge_df_write_byte(DF_I2C_CONFIGURATION, *i2c_configuration & ~I2C_CONFIGURATION_BCAST)) {
        return FUELGAUGE_BROADCAST_FAILED;
    }

    HAL_Delay(DF_WRITE_DELAY_MS);

    if (!fuelgauge_mac_read(DF_I2C_CONFIGURATION, i2c_configuration, 1)) {
        return FUELGAUGE_BROADCAST_FAILED;
    }
    return (*i2c_configuration & I2C_CONFIGURATION_BCAST) ? FUELGAUGE_BROADCAST_FAILED
                                                          : FUELGAUGE_BROADCAST_CLEARED;
}

fuelgauge_broadcast_result_t fuelgauge_disable_broadcast(uint8_t *i2c_configuration) {
    uint8_t value = 0;
    uint32_t timing = hi2c2.Instance->TIMINGR;

    i2c2_set_timing(I2C2_TIMING_100KHZ);
    fuelgauge_broadcast_result_t result = fuelgauge_clear_broadcast(&value);
    i2c2_set_timing(timing);

    if (i2c_configuration) {
        *i2c_configuration = value;
    }
    return result;
}

static bool fuelgauge_read_word(uint8_t command, uint16_t *value) {
    uint8_t data[2];
    if (fuelgauge_read(command, data, sizeof(data)) != HAL_OK) {
        return false;
    }
    *value = (uint16_t)(data[0] | (data[1] << 8));
    return true;
}

static bool fuelgauge_read_values(fuelgauge_status_t *status) {
    uint16_t soc = 0;
    uint16_t current = 0;
    uint8_t cells[4];

    /* DAStatus1() starts with the voltages of cell 1 and 2 */
    if (!fuelgauge_read_word(CMD_RELATIVE_STATE_OF_CHARGE, &soc)
        || !fuelgauge_read_word(CMD_VOLTAGE, &status->voltage)
        || !fuelgauge_read_word(CMD_CURRENT, &current)
        || !fuelgauge_mac_read(SUBCMD_DASTATUS1, cells, sizeof(cells))) {
        return false;
    }

    status->soc = soc > 100 ? 100 : (uint8_t)soc;
    status->current = (int16_t)current;
    status->cell1_voltage = (uint16_t)(cells[0] | (cells[1] << 8));
    status->cell2_voltage = (uint16_t)(cells[2] | (cells[3] << 8));
    return true;
}

bool fuelgauge_read_status(fuelgauge_status_t *status) {
    if (status == NULL) {
        return false;
    }

    uint32_t timing = hi2c2.Instance->TIMINGR;

    i2c2_set_timing(I2C2_TIMING_100KHZ);
    bool read = fuelgauge_read_values(status);
    i2c2_set_timing(timing);

    return read;
}
