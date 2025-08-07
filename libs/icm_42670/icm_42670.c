/**
 * @file icm_42670.c
 * @brief Driver for the TDK ICM-42670 6-axis IMU.
 *
 * This file provides functions to initialize and interface with the ICM-42670
 * Inertial Measurement Unit (IMU) using SPI on a Raspberry Pi Pico. It includes
 * functions for basic sensor reading, configuration, and a Kalman filter
 * implementation for sensor fusion to obtain stable pitch and roll angles.
 */

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "icm_42670.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

// --- Hardware Configuration ---
#define SPI_PORT spi0    ///< SPI peripheral to use (spi0 or spi1)
#define PIN_MISO 16      ///< SPI MISO (Master In, Slave Out) pin
#define PIN_CS   10      ///< SPI Chip Select pin (active low)
#define PIN_SCK  18      ///< SPI Serial Clock pin
#define PIN_MOSI 19      ///< SPI MOSI (Master Out, Slave In) pin

// --- Kalman Filter Globals ---
icm_42670_kalman_t pitch_filter;      ///< Kalman filter instance for the pitch axis
icm_42670_kalman_t roll_filter;       ///< Kalman filter instance for the roll axis
bool filters_initialized = false;   ///< Flag to ensure one-time initialization of filters
absolute_time_t last_time;          ///< Timestamp of the last Kalman filter update

// --- Function Prototypes for Kalman Filter ---
void icm_42670_kalman_init(icm_42670_kalman_t* kf, double Q_angle, double Q_bias, double R_measure);
double icm_42670_kalman_get_angle(icm_42670_kalman_t* kf, double newAngle, double newRate, double dt);
void icm_42670_kalman_init_struct();

/**
 * @brief Initializes the SPI peripheral for communication with the IMU.
 *
 * Configures the specified SPI port and sets up the GPIO pins (MISO, MOSI, SCK, CS)
 * for SPI functionality. The CS pin is configured as a standard output and
 * initialized to a high (inactive) state.
 */
void init_spi(void)
{
    spi_init(SPI_PORT, 400 * 1000); // Initialize SPI at 400kHz
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SIO); // Use standard I/O for CS
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Configure Chip Select pin
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1); // Deselect device
}

/**
 * @brief Initializes the ICM-42670 sensor with provided settings.
 *
 * Performs a soft reset, waits for the device to become ready, and then configures
 * the power management, gyroscope, accelerometer, and filter settings based on the
 * values in the initialization structure.
 *
 * @param icm_42670_init_struct Pointer to a struct containing the desired configuration.
 * @return uint8_t 1 on success, 0 on failure (timeout after reset).
 */
uint8_t init_icm_42670(icm_42670_t* icm_42670_init_struct)
{
    // Perform a soft reset on the device
    icm_42670_write_bank0_register(SIGNAL_PATH_RESET, 0x10);
    uint64_t start_time = time_us_64();

    // Wait for the reset-done interrupt flag with a 10ms timeout
    while ((icm_42670_read_bank0_register(INT_STATUS) & 0x10) == 0) {
        if (time_us_64() - start_time > 10000) {
            return 0; // Timeout failure
        }
        __asm volatile("nop");
    }

    // Configure power management and sensor modes (accel/gyro)
    icm_42670_write_bank0_register(PWR_MGMT0,
        icm_42670_init_struct->accel_lp_clk_sel << 6 |
        icm_42670_init_struct->idle << 3 |
        icm_42670_init_struct->gyro_mode << 2 |
        icm_42670_init_struct->accel_mode);

    // Configure gyroscope full-scale range (FS_SEL) and output data rate (ODR)
    icm_42670_write_bank0_register(GYRO_CONFIG0,
        icm_42670_init_struct->gyro_ui_fs_sel << 5 |
        icm_42670_init_struct->gyro_odr);

    // Configure accelerometer full-scale range (FS_SEL) and output data rate (ODR)
    icm_42670_write_bank0_register(ACCEL_CONFIG0,
        icm_42670_init_struct->accel_ui_fs_sel << 5 |
        icm_42670_init_struct->accel_odr);

    // Configure digital low-pass filters for all sensors
    icm_42670_write_bank0_register(TEMP_CONFIG0,
        icm_42670_init_struct->temp_filt_bw << 5);
    icm_42670_write_bank0_register(GYRO_CONFIG1,
        icm_42670_init_struct->gyro_ui_filt_bw);
    icm_42670_write_bank0_register(ACCEL_CONFIG1,
        icm_42670_init_struct->accel_ui_avg << 5 |
        icm_42670_init_struct->accel_ui_filt_bw);

    return 1; // Success
}

/**
 * @brief Configures the interrupt pins (INT1 and INT2).
 *
 * @param interrupt_config Pointer to a struct with interrupt settings (mode, polarity, drive).
 * @return uint8_t Always returns 0.
 */
uint8_t icm_42670_interrupt_config(icm_42670_int_t* interrupt_config)
{
    icm_42670_write_bank0_register(INT_CONFIG,
        interrupt_config->int2_mode << 5 |
        interrupt_config->int2_drive_circuit << 4 |
        interrupt_config->int2_polarity << 3 |
        interrupt_config->int1_mode << 2 |
        interrupt_config->int1_drive_circuit << 1 |
        interrupt_config->int1_polarity);
    return 0;
}

/**
 * @brief Configures the FIFO (First-In, First-Out) buffer.
 *
 * @param fifo_config Pointer to a struct with FIFO settings (mode, bypass).
 * @return uint8_t Always returns 0.
 */
uint8_t icm_42670_fifo_config(icm_42670_fifo_t* fifo_config)
{
    icm_42670_write_bank0_register(FIFO_CONFIG1,
        fifo_config->fifo_mode << 1 |
        fifo_config->fifo_bypass);
    return 0;
}

/**
 * @brief Configures the APEX (Advanced Pedometer and Event eXtensions) engine.
 *
 * Enables or disables features like the DMP, step counter, tilt detection, etc.
 *
 * @param apex_config Pointer to a struct with APEX feature settings.
 * @return uint8_t Always returns 0.
 */
uint8_t icm_42670_apex_config(icm_42670_apex_t* apex_config)
{
    icm_42670_write_bank0_register(APEX_CONFIG0,
        apex_config->dmp_power_save << 3 |
        apex_config->dmp_init << 2 |
        apex_config->dmp_mem_reset);

    icm_42670_write_bank0_register(APEX_CONFIG1,
        apex_config->smd_en << 6 |
        apex_config->freefall_en << 5 |
        apex_config->tilt_en << 4 |
        apex_config->pedometer_en << 3 |
        apex_config->dmp_odr);
    return 0;
}

/**
 * @brief Prints the status of key configuration registers for debugging.
 *
 * Reads and prints important registers to the console to verify the device state.
 */
void icm_42670_status(void)
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

/**
 * @brief Reads a single byte from a register in User Bank 0.
 *
 * @param reg The address of the register to read.
 * @return uint8_t The 8-bit value read from the register.
 */
uint8_t icm_42670_read_bank0_register(uint8_t reg)
{
    uint8_t tx_buf[2] = {0};
    uint8_t rx_buf[2] = {0};
    tx_buf[0] = reg | 0x80; // Set MSB to 1 for a read operation

    gpio_put(PIN_CS, 0); // Select device
    spi_write_read_blocking(SPI_PORT, tx_buf, rx_buf, 2);
    gpio_put(PIN_CS, 1); // Deselect device

    return rx_buf[1]; // Return the data byte
}

/**
 * @brief Writes a single byte to a register in User Bank 0.
 *
 * @param reg The address of the register to write to.
 * @param data The 8-bit value to write.
 * @return uint8_t Always returns 0.
 */
uint8_t icm_42670_write_bank0_register(uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2] = {reg & 0x7F, data}; // Clear MSB for a write operation
    gpio_put(PIN_CS, 0); // Select device
    spi_write_blocking(SPI_PORT, tx_buf, 2);
    gpio_put(PIN_CS, 1); // Deselect device
    return 0;
}

/**
 * @brief Reads a 16-bit signed value from two consecutive registers in User Bank 0.
 *
 * This is used for reading sensor data which is split across two 8-bit registers.
 *
 * @param reg The address of the first (high byte) register.
 * @return int16_t The combined 16-bit signed value.
 */
int16_t icm_42670_read_bank0_register_16(uint8_t reg)
{
    uint8_t high = icm_42670_read_bank0_register(reg);
    uint8_t low = icm_42670_read_bank0_register(reg + 1);
    return (int16_t)((high << 8) | low);
}

/**
 * @brief Reads a register from MREG1 (Memory Region 1).
 * @param reg The target register address within MREG1.
 * @return uint8_t The data read from the register.
 */
uint8_t icm_42670_read_mreg1_register(uint8_t reg)
{
    uint8_t data;
    icm_42670_write_bank0_register(BLK_SEL_R, 0x00); // Select MREG1 for reading
    icm_42670_write_bank0_register(MADDR_R, reg);   // Set address
    sleep_us(10);
    data = icm_42670_read_bank0_register(M_R);      // Read data
    sleep_us(10);
    return data;
}

/**
 * @brief Writes to a register in MREG1 (Memory Region 1).
 * @param reg The target register address within MREG1.
 * @param data The data to write.
 */
void icm_42670_write_mreg1_register(uint8_t reg, uint8_t data)
{
    icm_42670_write_bank0_register(BLK_SEL_W, 0x00); // Select MREG1 for writing
    icm_42670_write_bank0_register(MADDR_W, reg);   // Set address
    icm_42670_write_bank0_register(M_W, data);      // Write data
    sleep_us(10);
}

// NOTE: The comments for MREG2 and MREG3 functions follow the same pattern as MREG1.

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


/**
 * @brief Reads raw data from all sensors (accel, gyro, temp).
 *
 * @param data Pointer to a struct where the raw sensor data will be stored.
 */
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

/**
 * @brief Reads the temperature sensor and converts it to Celsius.
 *
 * @return float Temperature in degrees Celsius.
 */
float icm_42670_read_temperature_celsius()
{
    int16_t raw_temp = icm_42670_read_bank0_register_16(TEMP_DATA1);
    // Formula from datasheet: Temp_C = (RAW_TEMP / 132.48) + 25
    return (raw_temp / 132.48f) + 25.0f;
}

/**
 * @brief Reads the temperature sensor and converts it to Kelvin.
 *
 * @return float Temperature in Kelvin.
 */
float icm_42670_read_temperature_kelvin()
{
    return icm_42670_read_temperature_celsius() + 273.15f;
}

/**
 * @brief Reads the temperature sensor and converts it to Fahrenheit.
 *
 * @return float Temperature in degrees Fahrenheit.
 */
float icm_42670_read_temperature_fahrenheit()
{
    return (icm_42670_read_temperature_celsius() * 1.8f) + 32.0f;
}

/**
 * @brief Reads raw data from the gyroscope.
 *
 * @param data Pointer to a struct where the raw gyro data will be stored.
 */
void icm_42670_read_gyro(icm_42670_gyro_data_t* data)
{
    data->gx = icm_42670_read_bank0_register_16(GYRO_DATA_X1);
    data->gy = icm_42670_read_bank0_register_16(GYRO_DATA_Y1);
    data->gz = icm_42670_read_bank0_register_16(GYRO_DATA_Z1);
}

/**
 * @brief Reads raw data from the accelerometer.
 *
 * @param data Pointer to a struct where the raw accel data will be stored.
 */
void icm_42670_read_accel(icm_42670_accel_data_t* data)
{
    data->ax = icm_42670_read_bank0_register_16(ACCEL_DATA_X1);
    data->ay = icm_42670_read_bank0_register_16(ACCEL_DATA_Y1);
    data->az = icm_42670_read_bank0_register_16(ACCEL_DATA_Z1);
}

/**
 * @brief Initializes a Kalman filter structure with given tuning parameters.
 *
 * @param kf Pointer to the Kalman filter instance to initialize.
 * @param Q_angle Process noise covariance for the angle.
 * @param Q_bias Process noise covariance for the gyro bias.
 * @param R_measure Measurement noise covariance.
 */
void icm_42670_kalman_init(icm_42670_kalman_t* kf, double Q_angle, double Q_bias, double R_measure)
{
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R_measure = R_measure;

    kf->angle = 0.0;
    kf->bias = 0.0;
    kf->rate = 0.0;

    // Initialize the error covariance matrix P
    kf->P[0][0] = 0.0;
    kf->P[0][1] = 0.0;
    kf->P[1][0] = 0.0;
    kf->P[1][1] = 0.0;
}

/**
 * @brief Initializes the global pitch and roll Kalman filters if they haven't been already.
 *
 * This function is a wrapper that ensures the filters are initialized only once.
 * It uses predefined tuning values for the filters.
 */
void icm_42670_kalman_init_struct()
{
    if (!filters_initialized) {
        icm_42670_kalman_init(&pitch_filter, 0.001, 0.003, 0.03);
        icm_42670_kalman_init(&roll_filter, 0.001, 0.003, 0.03);
        last_time = get_absolute_time();
        filters_initialized = true;
    }
}

/**
 * @brief Performs one complete update cycle for both pitch and roll Kalman filters.
 *
 * Reads sensor data, converts it to physical units, calculates angles from the
 * accelerometer, and then updates both the pitch and roll Kalman filter states.
 */
void icm_42670_kalman_update()
{
    icm_42670_kalman_init_struct(); // Ensure filters are initialized

    icm_42670_all_sensors_data_t data;
    icm_42670_read_all_sensors(&data);

    // Convert raw data to physical units (assuming FS_SEL = 2000dps and 16g)
    float ax_g = data.ax / 2048.0f;
    float ay_g = data.ay / 2048.0f;
    float az_g = data.az / 2048.0f;
    float gx_dps = data.gx / 16.4f;
    float gy_dps = data.gy / 16.4f;
    float gz_dps = data.gz / 16.4f;

    // Calculate time delta (dt) in seconds
    absolute_time_t current_time = get_absolute_time();
    double dt = (double)absolute_time_diff_us(last_time, current_time) / 1000000.0;
    last_time = current_time;

    // Calculate angles from accelerometer data
    double accel_pitch = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0 / M_PI;
    double accel_roll = atan2(-ax_g, az_g) * 180.0 / M_PI;

    // Update the filters with new data
    icm_42670_kalman_get_angle(&roll_filter, accel_roll, gx_dps, dt);
    icm_42670_kalman_get_angle(&pitch_filter, accel_pitch, gy_dps, dt);
}

/**
 * @brief Executes one step of the Kalman filter algorithm.
 *
 * Fuses the angle from the accelerometer with the rate from the gyroscope to
 * produce a smoothed, more accurate angle estimate.
 *
 * @param kf Pointer to the Kalman filter instance.
 * @param newAngle The angle calculated from the accelerometer (in degrees).
 * @param newRate The angular rate from the gyroscope (in degrees/sec).
 * @param dt The time delta since the last update (in seconds).
 * @return double The new filtered angle.
 */
double icm_42670_kalman_get_angle(icm_42670_kalman_t* kf, double newAngle, double newRate, double dt)
{
    // Predict state
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    // Update error covariance
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += dt * kf->Q_bias;

    // Calculate Kalman Gain
    double S = kf->P[0][0] + kf->R_measure;
    double K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // Update estimate with measurement
    double y = newAngle - kf->angle;
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;

    // Update the error covariance matrix
    double P00_temp = kf->P[0][0];
    double P01_temp = kf->P[0][1];
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

/**
 * @brief Convenience function to run an update and get the filtered angles.
 *
 * @param angles Pointer to a struct to store the resulting pitch and roll.
 */
void icm_42670_kalman_get_angles_autoupdate(icm_42670_angles_data_t* angles)
{
    icm_42670_kalman_init_struct(); // Ensure filters are ready
    icm_42670_kalman_update();      // Run a new update cycle
    icm_42670_kalman_get_angles(angles); // Populate the struct
}

/**
 * @brief Gets the most recent filtered pitch and roll angles.
 *
 * This function retrieves the current state of the global Kalman filters without
 * running a new update cycle.
 *
 * @param angles Pointer to a struct to store the pitch and roll.
 */
void icm_42670_kalman_get_angles(icm_42670_angles_data_t* angles)
{
    angles->pitch = pitch_filter.angle;
    angles->roll = roll_filter.angle;
}

/**
 * @brief Checks if the DMP (Digital Motion Processor) is running.
 *
 * @return uint8_t 1 if running, 0 if not.
 */
uint8_t icm_42670_dmp_is_running(void)
{
    // Checks the DMP_IDLE bit in the APEX_DATA3 register
    return (icm_42670_read_bank0_register(APEX_DATA3) & 0x04) >> 2;
}

/**
 * @brief Reads the total step count from the APEX pedometer.
 *
 * @return uint16_t The number of steps detected.
 */
uint16_t icm_42670_apex_step_count(void)
{
    uint8_t high = icm_42670_read_bank0_register(APEX_DATA1);
    uint8_t low = icm_42670_read_bank0_register(APEX_DATA0);
    return (uint16_t)((high << 8) | low);
}

/**
 * @brief Reads the step cadence (walking speed).
 *
 * The unit is 2 steps/sec. A value of 0 indicates a cadence of less than 0.5 Hz.
 *
 * @return uint8_t The current step cadence.
 */
uint8_t icm_42670_apex_step_cadence(void)
{
    return icm_42670_read_bank0_register(APEX_DATA2);
}

/**
 * @brief Reads the current user activity detected by the pedometer.
 *
 * @return uint8_t The activity status (0: Unknown, 1: Walk, 2: Run).
 */
uint8_t icm_42670_apex_pedometer_activity(void)
{
    // Masks the lower two bits of APEX_DATA3
    return icm_42670_read_bank0_register(APEX_DATA3) & 0x03;
}