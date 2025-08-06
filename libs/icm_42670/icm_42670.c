#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "icm_42670.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   10
#define PIN_SCK  18
#define PIN_MOSI 19

icm_42670_kalman_t pitch_filter;
icm_42670_kalman_t  roll_filter;
bool filters_initialized = false;
absolute_time_t last_time;

void icm_42670_kalman_init(icm_42670_kalman_t* kf, double Q_angle, double Q_bias, double R_measure);
double icm_42670_kalman_get_angle(icm_42670_kalman_t* kf, double newAngle, double newRate, double dt);
void icm_42670_kalman_init_struct();

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

uint8_t init_icm_42670(icm_42670_t* icm_42670_init_struct)
{
    //soft reset
    icm_42670_write_bank0_register(SIGNAL_PATH_RESET, 0x10);
    uint64_t start_time = time_us_64();
    while((icm_42670_read_bank0_register(INT_STATUS) & 0x10) == 0) {
        if(time_us_64() - start_time > 10000) {
            return 0; 
        }
        __asm volatile ("nop");
    }

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

    return 1;
}

uint8_t icm_42670_interrupt_config(icm_42670_int_t* interrupt_config)
{
    icm_42670_write_bank0_register(INT_CONFIG, 
        interrupt_config->int2_mode << 5 |
        interrupt_config->int2_drive_circuit << 4 |
        interrupt_config->int2_polarity << 3 |
        interrupt_config->int1_mode << 2 |
        interrupt_config->int1_drive_circuit << 1 |
        interrupt_config->int1_polarity
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

uint8_t icm_42670_read_bank0_register(uint8_t reg) 
{
    uint8_t tx_buf[2] = {0};
    uint8_t rx_buf[2] = {0};
    tx_buf[0] = reg | 0x80;

    gpio_put(PIN_CS, 0);
    spi_write_read_blocking(SPI_PORT, tx_buf, rx_buf, 2);
    gpio_put(PIN_CS, 1);

    return rx_buf[1];
}

uint8_t icm_42670_write_bank0_register(uint8_t reg, uint8_t data) 
{
    uint8_t tx_buf[2] = {reg & 0x7F, data};
    uint8_t rx_buf[2] = {0};
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, tx_buf, 2);
    gpio_put(PIN_CS, 1);
    return 0;
}

int16_t icm_42670_read_bank0_register_16(uint8_t reg) 
{
    uint8_t high = icm_42670_read_bank0_register(reg);
    uint8_t low = icm_42670_read_bank0_register(reg + 1);
    return (int16_t)((high << 8) | low);
}


uint8_t icm_42670_read_mreg1_register(uint8_t reg)
{
    uint8_t data;
    icm_42670_write_bank0_register(BLK_SEL_R, 0x00);
    icm_42670_write_bank0_register(MADDR_R, reg);
    sleep_us(10);
    data = icm_42670_read_bank0_register(M_R);
    sleep_us(10);
    return data;
}

void icm_42670_write_mreg1_register(uint8_t reg, uint8_t data)
{
    icm_42670_write_bank0_register(BLK_SEL_W, 0x00);
    icm_42670_write_bank0_register(MADDR_W, reg);
    icm_42670_write_bank0_register(M_W, data);
    sleep_us(10);
}

uint8_t icm_42670_read_mreg2_register(uint8_t reg)
{
    uint8_t data;
    icm_42670_write_bank0_register(BLK_SEL_R, 0x28);
    icm_42670_write_bank0_register(MADDR_R, reg);
    sleep_us(10);
    data = icm_42670_read_bank0_register(M_R);
    sleep_us(10);
    return data;
}

void icm_42670_write_mreg2_register(uint8_t reg, uint8_t data)
{
    icm_42670_write_bank0_register(BLK_SEL_W, 0x28);
    icm_42670_write_bank0_register(MADDR_W, reg);
    icm_42670_write_bank0_register(M_W, data);
    sleep_us(10);
}

uint8_t icm_42670_read_mreg3_register(uint8_t reg)
{
    uint8_t data;
    icm_42670_write_bank0_register(BLK_SEL_R, 0x50);
    icm_42670_write_bank0_register(MADDR_R, reg);
    sleep_us(10);
    data = icm_42670_read_bank0_register(M_R);
    sleep_us(10);
    return data;
}

void icm_42670_write_mreg3_register(uint8_t reg, uint8_t data)
{
    icm_42670_write_bank0_register(BLK_SEL_W, 0x50);
    icm_42670_write_bank0_register(MADDR_W, reg);
    icm_42670_write_bank0_register(M_W, data);
    sleep_us(10);
}

void icm_42670_read_all_sensors(icm_42670_all_sensors_data_t* data) 
{
    data->ax = icm_42670_read_bank0_register_16(ACCEL_DATA_X1);
    data->ay = icm_42670_read_bank0_register_16(ACCEL_DATA_Y1);
    data->az = icm_42670_read_bank0_register_16(ACCEL_DATA_Z1);
    data->gx = icm_42670_read_bank0_register_16(GYRO_DATA_X1);
    data->gy = icm_42670_read_bank0_register_16(GYRO_DATA_Y1);
    data->gz = icm_42670_read_bank0_register_16(GYRO_DATA_Z1);
    data->temperature = icm_42670_read_bank0_register_16(TEMP_DATA1);
}

float icm_42670_read_temperature_celsius() 
{
    int16_t raw_temp =icm_42670_read_bank0_register_16(TEMP_DATA1);
    return (raw_temp / 132.48f) + 25.0f;
}

float icm_42670_read_temperature_kelvin() 
{
    return icm_42670_read_temperature_celsius() + 273.15f;
}

float icm_42670_read_temperature_fahrenheit() 
{
    return (icm_42670_read_temperature_celsius() * 1.8) + 32;
}

void icm_42670_read_gyro(icm_42670_gyro_data_t* data)
{
    data->gx = icm_42670_read_bank0_register_16(GYRO_DATA_X1);
    data->gy = icm_42670_read_bank0_register_16(GYRO_DATA_Y1);
    data->gz = icm_42670_read_bank0_register_16(GYRO_DATA_Z1);
}

void icm_42670_read_accel(icm_42670_accel_data_t* data)
{
    data->ax = icm_42670_read_bank0_register_16(ACCEL_DATA_X1);
    data->ay = icm_42670_read_bank0_register_16(ACCEL_DATA_Y1);
    data->az = icm_42670_read_bank0_register_16(ACCEL_DATA_Z1);
}

void icm_42670_kalman_init(icm_42670_kalman_t* kf, double Q_angle, double Q_bias, double R_measure) 
{
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R_measure = R_measure;

    kf->angle = 0.0;
    kf->bias = 0.0;
    kf->rate = 0.0;

    kf->P[0][0] = 0.0;
    kf->P[0][1] = 0.0;
    kf->P[1][0] = 0.0;
    kf->P[1][1] = 0.0;
}

void icm_42670_kalman_init_struct() 
{
    if (!filters_initialized) 
    {
        icm_42670_kalman_init(&pitch_filter, 0.001, 0.003, 0.03);
        icm_42670_kalman_init(&roll_filter, 0.001, 0.003, 0.03);
        last_time = get_absolute_time();
        filters_initialized = true;
    }
}

void icm_42670_kalman_update() 
{
    icm_42670_kalman_init_struct();

    icm_42670_all_sensors_data_t data;
    icm_42670_read_all_sensors(&data);
    float ax_g = data.ax / 2048.0f;
    float ay_g = data.ay / 2048.0f;
    float az_g = data.az / 2048.0f;
    float gx_dps = data.gx / 16.4f;
    float gy_dps = data.gy / 16.4f;
    float gz_dps = data.gz / 16.4f;

    absolute_time_t current_time = get_absolute_time();
    double dt = (double)absolute_time_diff_us(last_time, current_time) / 1000000.0;
    last_time = current_time;

    double accel_pitch = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0 / M_PI;
    double accel_roll = atan2(-ax_g, az_g) * 180.0 / M_PI;

    icm_42670_kalman_get_angle(&roll_filter, accel_roll, gx_dps, dt);
    icm_42670_kalman_get_angle(&pitch_filter, accel_pitch, gy_dps, dt);
}

double icm_42670_kalman_get_angle(icm_42670_kalman_t* kf, double newAngle, double newRate, double dt) 
{
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += dt * kf->Q_bias;

    double S = kf->P[0][0] + kf->R_measure;
    double K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    double y = newAngle - kf->angle;
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;

    double P00_temp = kf->P[0][0];
    double P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

void icm_42670_kalman_get_angles_autoupdate(icm_42670_angles_data_t* angles)
{
    icm_42670_all_sensors_data_t data;
    icm_42670_read_all_sensors(&data);
    float ax_g = data.ax / 2048.0f;
    float ay_g = data.ay / 2048.0f;
    float az_g = data.az / 2048.0f;
    float gx_dps = data.gx / 16.4f;
    float gy_dps = data.gy / 16.4f;
    float gz_dps = data.gz / 16.4f;

    icm_42670_kalman_init_struct();
    icm_42670_kalman_update(ax_g, ay_g, az_g, gx_dps, gy_dps);
    icm_42670_kalman_get_angles(angles);
}

void icm_42670_kalman_get_angles(icm_42670_angles_data_t* angles) 
{
    angles->pitch = pitch_filter.angle;
    angles->roll = roll_filter.angle;
}