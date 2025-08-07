/**
 * @file icm_42670.h
 * @brief Public API for the ICM-42670 IMU driver.
 * (Full documentation is in the .c file)
 */

#ifndef ICM_42670_H
#define ICM_42670_H

#include <stdint.h>
#include "icm_42670_types.h"
#include "icm_42670_registers.h"

// --- Initialization and Configuration ---
void init_spi(void);                                              // Initializes the SPI peripheral.
uint8_t init_icm_42670(icm_42670_t* icm_42670_init_struct);       // Initializes the IMU sensor. Returns 1 on success.
void icm_42670_status(void);                                   // Prints key register values for debugging.
uint8_t icm_42670_interrupt_config(icm_42670_int_t* interrupt_config); // Configures the INT1/INT2 interrupt pins.
uint8_t icm_42670_fifo_config(icm_42670_fifo_t* fifo_config);       // Configures the sensor's FIFO buffer.
uint8_t icm_42670_apex_config(icm_42670_apex_t* apex_config);       // Configures APEX motion features (pedometer, etc.).

// --- Low-Level Register Access ---
uint8_t icm_42670_read_bank0_register(uint8_t reg);                // Reads a single register from User Bank 0.
int16_t icm_42670_read_bank0_register_16(uint8_t reg);             // Reads a 16-bit value (two registers) from Bank 0.
uint8_t icm_42670_write_bank0_register(uint8_t reg, uint8_t data); // Writes to a single register in User Bank 0.
uint8_t icm_42670_read_mreg1_register(uint8_t reg);                // Reads a register from MREG1.
void icm_42670_write_mreg1_register(uint8_t reg, uint8_t data);    // Writes to a register in MREG1.
uint8_t icm_42670_read_mreg2_register(uint8_t reg);                // Reads a register from MREG2.
void icm_42670_write_mreg2_register(uint8_t reg, uint8_t data);    // Writes to a register in MREG2.
uint8_t icm_42670_read_mreg3_register(uint8_t reg);                // Reads a register from MREG3.
void icm_42670_write_mreg3_register(uint8_t reg, uint8_t data);    // Writes to a register in MREG3.

// --- High-Level Sensor Reading ---
void icm_42670_read_all_sensors(icm_42670_all_sensors_data_t* data); // Reads raw data from all sensors into a struct.
float icm_42670_read_temperature_celsius();                        // Reads temperature in Celsius.
float icm_42670_read_temperature_kelvin();                         // Reads temperature in Kelvin.
float icm_42670_read_temperature_fahrenheit();                     // Reads temperature in Fahrenheit.
void icm_42670_read_gyro(icm_42670_gyro_data_t* data);              // Reads raw gyroscope data.
void icm_42670_read_accel(icm_42670_accel_data_t* data);            // Reads raw accelerometer data.

// --- Kalman Filter for Orientation ---
void icm_42670_kalman_update(void);                                   // Runs a single Kalman filter update cycle.
void icm_42670_kalman_get_angles(icm_42670_angles_data_t* angles);   // Gets the latest filtered pitch and roll angles.
void icm_42670_kalman_get_angles_autoupdate(icm_42670_angles_data_t* angles); // Updates filter then gets new angles.

// --- APEX Motion Feature Access ---
uint8_t icm_42670_dmp_is_running(void);                            // Checks if the DMP is active.
uint16_t icm_42670_apex_step_count(void);                          // Gets the total step count.
uint8_t icm_42670_apex_step_cadence(void);                         // Gets the current step cadence (walking speed).
uint8_t icm_42670_apex_pedometer_activity(void);                   // Gets the current detected activity (walk, run, etc.).

#endif // ICM_42670_H