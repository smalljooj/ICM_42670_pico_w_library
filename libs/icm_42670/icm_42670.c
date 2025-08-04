#include <stdio.h>
#include "icm_42670.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   10
#define PIN_SCK  18
#define PIN_MOSI 19

void init_spi(void)
{
    spi_init(SPI_PORT, 400*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
}

void init_icm_42670(icm_42670_t* icm_42670_init_struct)
{
    icm_42670_write_bank0_register(PWR_MGMT0, 
        icm_42670_init_struct->accel_lp_clk_sel << 6 |
        icm_42670_init_struct->idle << 3 |
        icm_42670_init_struct->gyro_mode << 2 |
        icm_42670_init_struct->accel_mode  
    );
    // set the number of samples and ui interface to the gyro
    icm_42670_write_bank0_register(GYRO_CONFIG0, 
        icm_42670_init_struct->gyro_ui_fs_sel << 5 |
        icm_42670_init_struct->gyro_odr 
    );

    // set the number of samples and ui interface to the accel
    icm_42670_write_bank0_register(ACCEL_CONFIG0, 
        icm_42670_init_struct->accel_ui_fs_sel << 5 |
        icm_42670_init_struct->accel_odr 
    );

    // filters for temperature, gyro and accel sensors
    icm_42670_write_bank0_register(TEMP_CONFIG0, 
        icm_42670_init_struct->temp_filt_bw << 5
    );
    icm_42670_write_bank0_register(GYRO_CONFIG1, 
        icm_42670_init_struct->gyro_ui_filt_bw 
    );
    icm_42670_write_bank0_register(ACCEL_CONFIG1, 
        icm_42670_init_struct->accel_ui_avg << 5 |
        icm_42670_init_struct->accel_ui_filt_bw 
    );
}

uint8_t icm_42670_status(void)
{
    printf("mclk: %02x\n", icm_42670_read_bank0_register(MCLK_RDY));
    printf("device config: %02x\n", icm_42670_read_bank0_register(DEVICE_CONFIG));
    printf("signal path: %02x\n", icm_42670_read_bank0_register(SIGNAL_PATH_RESET));
    printf("drive config: %02x\n", icm_42670_read_bank0_register(DRIVE_CONFIG1));
    printf("interrupt config: %02x\n", icm_42670_read_bank0_register(INT_CONFIG));
    printf("pwr mgmt0: %02x\n", icm_42670_read_bank0_register(PWR_MGMT0));
    printf("gyro config0: %02x\n", icm_42670_read_bank0_register(GYRO_CONFIG0));
    printf("accel config0: %02x\n", icm_42670_read_bank0_register(ACCEL_CONFIG0));
    printf("temp config0: %02x\n", icm_42670_read_bank0_register(TEMP_CONFIG0));
    printf("who am i: %02x\n", icm_42670_read_bank0_register(WHO_AM_I));
}

uint8_t icm_42670_read_bank0_register(uint8_t reg) {
    uint8_t tx_buf[2] = {0};
    uint8_t rx_buf[2] = {0};
    tx_buf[0] = reg | 0x80;

    gpio_put(PIN_CS, 0);
    spi_write_read_blocking(SPI_PORT, tx_buf, rx_buf, 2);
    gpio_put(PIN_CS, 1);

    return rx_buf[1];
}

uint8_t icm_42670_write_bank0_register(uint8_t reg, uint8_t data) {
    uint8_t tx_buf[2] = {reg & 0x7F, data};
    uint8_t rx_buf[2] = {0};
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, tx_buf, 2);
    gpio_put(PIN_CS, 1);
    return 0;
}

int16_t icm_42670_read_bank0_register_16(uint8_t reg) {
    uint8_t high = icm_42670_read_bank0_register(reg);
    uint8_t low = icm_42670_read_bank0_register(reg + 1);
    return (int16_t)((high << 8) | low);
}

void icm_42670_read_all_sensors(icm_42670_all_sensors_data* data) {
    data->ax = icm_42670_read_bank0_register_16(ACCEL_DATA_X1);
    data->ay = icm_42670_read_bank0_register_16(ACCEL_DATA_Y1);
    data->az = icm_42670_read_bank0_register_16(ACCEL_DATA_Z1);
    data->gx = icm_42670_read_bank0_register_16(GYRO_DATA_X1);
    data->gy = icm_42670_read_bank0_register_16(GYRO_DATA_Y1);
    data->gz = icm_42670_read_bank0_register_16(GYRO_DATA_Z1);
    data->temperature = icm_42670_read_bank0_register_16(TEMP_DATA1);
}

float icm_42670_read_temperature() {
    int16_t raw_temp =icm_42670_read_bank0_register_16(TEMP_DATA1);
    return (raw_temp / 132.48f) + 25.0f;
}