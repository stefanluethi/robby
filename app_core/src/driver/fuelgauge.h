#ifndef FUELGAUGE_H
#define FUELGAUGE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** What the gauge reports about the battery */
typedef struct {
    /** Relative state of charge in % */
    uint8_t soc;
    /** Pack voltage in mV */
    uint16_t voltage;
    /** Cell voltages in mV */
    uint16_t cell1_voltage;
    uint16_t cell2_voltage;
    /** Current in mA, positive while charging, negative while discharging */
    int16_t current;
} fuelgauge_status_t;

typedef enum {
    /** BROADCAST mode was already off, nothing was written */
    FUELGAUGE_BROADCAST_OFF,
    /** BROADCAST mode was on, it got cleared and the read back confirms it */
    FUELGAUGE_BROADCAST_CLEARED,
    /** The gauge is SEALED, its data flash can't be accessed */
    FUELGAUGE_BROADCAST_SEALED,
    /** The gauge doesn't acknowledge its address */
    FUELGAUGE_BROADCAST_NO_GAUGE,
    /** A transfer failed or the read back still shows BROADCAST mode */
    FUELGAUGE_BROADCAST_FAILED,
} fuelgauge_broadcast_result_t;

/**
 * @brief Initialize the fuel gauge (if necessary).
 * @return true if device responds, false otherwise.
 */
bool fuelgauge_init(void);

/**
 * @brief Read the relative state of charge from the BQ28Z620 fuel gauge.
 * @param[out] soc Pointer to store the state of charge (0-100%).
 * @return true if read was successful, false otherwise.
 */
bool fuelgauge_get_soc(uint8_t *soc);

/**
 * @brief Read the momentary current consumption from the BQ28Z620 fuel gauge.
 * @param[out] current Pointer to store the current in mA (positive for charging, negative for discharging).
 * @return true if read was successful, false otherwise.
 */
bool fuelgauge_get_current(int16_t *current);

/**
 * @brief Read the voltage of Cell 1 from the BQ28Z620 fuel gauge.
 * @param[out] voltage Pointer to store the cell 1 voltage in mV.
 * @return true if read was successful, false otherwise.
 */
bool fuelgauge_get_cell1_voltage(uint16_t *voltage);

/**
 * @brief Read the voltage of Cell 2 from the BQ28Z620 fuel gauge.
 * @param[out] voltage Pointer to store the cell 2 voltage in mV.
 * @return true if read was successful, false otherwise.
 */
bool fuelgauge_get_cell2_voltage(uint16_t *voltage);

/**
 * @brief Turn off the gauge's BROADCAST mode in its data flash (I2C Configuration, 0x4602).
 *
 * In BROADCAST mode the gauge writes ChargingCurrent() to the charger address 0xD4 every 15 s,
 * which is also the LSM6DSO's address and overwrites its FIFO_CTRL4. The data flash is only
 * written when BROADCAST mode is on. Switches I2C2 to ~100 kHz for the access and restores the
 * previous timing afterwards, so nothing else may use I2C2 meanwhile.
 * @param[out] i2c_configuration Optional, last value read from I2C Configuration.
 * @return What was found and done.
 */
fuelgauge_broadcast_result_t fuelgauge_disable_broadcast(uint8_t *i2c_configuration);

/**
 * @brief Read the state of charge, the voltages and the current in one access.
 *
 * Switches I2C2 to ~100 kHz for the access and restores the previous timing afterwards, so
 * nothing else may use I2C2 meanwhile. Takes a few milliseconds.
 * @param[out] status The values read, only valid if true is returned.
 * @return true if all values were read.
 */
bool fuelgauge_read_status(fuelgauge_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* FUELGAUGE_H */
