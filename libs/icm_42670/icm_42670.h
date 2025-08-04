#ifndef ICM_42670
#define ICM_42670

#include <stdint.h>
#include "icm_42670_types.h"
#include "icm_42670_registers.h"

void init_spi(void);
void init_icm_42670(icm_42670_t* icm_42670_init_struct);
uint8_t icm_42670_status(void);
uint8_t icm_42670_read_bank0_register(uint8_t reg);
int16_t icm_42670_read_bank0_register_16(uint8_t reg);
uint8_t icm_42670_write_bank0_register(uint8_t reg, uint8_t data);
void icm_42670_read_all_sensors(icm_42670_all_sensors_data* data);
float icm_42670_read_temperature(); 

#endif