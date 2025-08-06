#ifndef ICM_42670
#define ICM_42670

#include <stdint.h>
#include "icm_42670_types.h"
#include "icm_42670_registers.h"

void init_spi(void);
uint8_t init_icm_42670(icm_42670_t* icm_42670_init_struct);
uint8_t icm_42670_status(void);

uint8_t icm_42670_interrupt_config(icm_42670_int_t* interrupt_config);

uint8_t icm_42670_read_bank0_register(uint8_t reg);
int16_t icm_42670_read_bank0_register_16(uint8_t reg);
uint8_t icm_42670_write_bank0_register(uint8_t reg, uint8_t data);
uint8_t icm_42670_read_mreg1_register(uint8_t reg);
void icm_42670_write_mreg1_register(uint8_t reg, uint8_t data);
uint8_t icm_42670_read_mreg2_register(uint8_t reg);
void icm_42670_write_mreg2_register(uint8_t reg, uint8_t data);
uint8_t icm_42670_read_mreg3_register(uint8_t reg);
void icm_42670_write_mreg3_register(uint8_t reg, uint8_t data);

void icm_42670_read_all_sensors(icm_42670_all_sensors_data_t* data);

float icm_42670_read_temperature_celsius(); 
float icm_42670_read_temperature_kelvin();
float icm_42670_read_temperature_fahrenheit();

void icm_42670_read_gyro(icm_42670_gyro_data_t* data); 
void icm_42670_read_accel(icm_42670_accel_data_t* data); 

void icm_42670_kalman_update();
void icm_42670_kalman_get_angles(icm_42670_angles_data_t* angles);
void icm_42670_kalman_get_angles_autoupdate(icm_42670_angles_data_t* angles);

#endif