#ifndef FUELGAUGE_H
#define FUELGAUGE_H

#include <stdint.h>
#include <stdbool.h>

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

#endif /* FUELGAUGE_H */
