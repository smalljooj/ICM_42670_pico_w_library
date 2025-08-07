/**
 * @file icm_42670_types.h
 * @brief Defines data structures and constants for the ICM-42670 driver.
 */

#ifndef ICM_42670_TYPES_H
#define ICM_42670_TYPES_H

// Holds state and tuning parameters for a Kalman filter.
typedef struct {
    double Q_angle;    // Process noise covariance for angle
    double Q_bias;     // Process noise covariance for bias
    double R_measure;  // Measurement noise covariance

    double angle;      // The angle calculated by the filter
    double bias;       // The gyro bias calculated by the filter
    double rate;       // The corrected rate
    double P[2][2];    // Error covariance matrix
} icm_42670_kalman_t;

// --- Data Structures ---
typedef struct {
    float gx, gy, gz;  // Stores raw gyroscope data.
} icm_42670_gyro_data_t;

typedef struct {
    float ax, ay, az;  // Stores raw accelerometer data.
} icm_42670_accel_data_t;

typedef struct {
    float pitch, roll; // Stores filtered pitch and roll angles.
} icm_42670_angles_data_t;

// Stores data from all sensors.
typedef struct {
    float ax, ay, az;    // Accelerometer
    float gx, gy, gz;    // Gyroscope
    int16_t temperature; // Temperature
} icm_42670_all_sensors_data_t;


// --- Sensor Configuration Enums ---

// Gyroscope operating modes.
typedef enum {
    GYRO_OFF = 0,
    GYRO_STANDBY = 1,
    GYRO_LN_MODE = 3 // Low-Noise mode
} GYRO_MODE;

// Accelerometer operating modes.
typedef enum {
    ACCEL_OFF = 0,
    ACCEL_LP_MODE = 2, // Low-Power mode
    ACCEL_LN_MODE = 3  // Low-Noise mode
} ACCEL_MODE;

// Idle mode (low-power state).
typedef enum {
    IDLE_OFF = 0,
    IDLE_ON = 1
} IDLE;

// Clock source for accelerometer low-power mode.
typedef enum {
    WAKE_UP_OSCILLATOR = 0,
    RC_OSCILLATOR = 1
} ACCEL_LP_CLK_SEL;

// Gyroscope full-scale range (degrees per second).
typedef enum {
    GYRO_UI_2000_DPS = 0,
    GYRO_UI_1000_DPS = 1,
    GYRO_UI_500_DPS = 2,
    GYRO_UI_250_DPS = 3
} GYRO_UI_FS_SEL;

// Gyroscope output data rate (ODR).
typedef enum {
    GYRO_ODR_1600HZ = 5,
    GYRO_ODR_800HZ = 6,
    GYRO_ODR_400HZ = 7,
    GYRO_ODR_200HZ = 8,
    GYRO_ODR_100HZ = 9,
    GYRO_ODR_50HZ = 10,
    GYRO_ODR_25HZ = 11,
    GYRO_ODR_12_5HZ = 12,
} GYRO_ODR;

// Accelerometer full-scale range (g-force).
typedef enum {
    ACCEL_UI_16_G = 0,
    ACCEL_UI_8_G = 1,
    ACCEL_UI_4_G = 2,
    ACCEL_UI_2_G = 3
} ACCEL_UI_FS_SEL;

// Accelerometer output data rate (ODR).
typedef enum {
    ACCEL_ODR_1600HZ = 5,
    ACCEL_ODR_800HZ = 6,
    ACCEL_ODR_400HZ = 7,
    ACCEL_ODR_200HZ = 8,
    ACCEL_ODR_100HZ = 9,
    ACCEL_ODR_50HZ = 10,
    ACCEL_ODR_25HZ = 11,
    ACCEL_ODR_12_5HZ = 12,
    ACCEL_ODR_6_25HZ = 13,
    ACCEL_ODR_3_125HZ = 14,
    ACCEL_ODR_1_5625HZ = 15,
} ACCEL_ODR;

// Temperature sensor filter bandwidth.
typedef enum {
    TEMP_FILT_BYPASSED = 0,
    TEMP_FILT_180_HZ = 1,
    TEMP_FILT_72_HZ = 2,
    TEMP_FILT_34_HZ = 3,
    TEMP_FILT_16_HZ = 4,
    TEMP_FILT_8_HZ = 5,
    TEMP_FILT_4_HZ = 6,
} TEMP_FILT_BW;

// Gyroscope filter bandwidth.
typedef enum {
    GYRO_FILT_BYPASSED = 0,
    GYRO_FILT_180_HZ = 1,
    GYRO_FILT_121_HZ = 2,
    GYRO_FILT_73_HZ = 3,
    GYRO_FILT_53_HZ = 4,
    GYRO_FILT_34_HZ = 5,
    GYRO_FILT_25_HZ = 6,
    GYRO_FILT_16_HZ = 7,
} GYRO_UI_FILT_BW;

// Accelerometer averaging filter settings.
typedef enum {
    ACCEL_AVG_2X = 0,
    ACCEL_AVG_4X = 1,
    ACCEL_AVG_8X = 2,
    ACCEL_AVG_16X = 3,
    ACCEL_AVG_32X = 4,
    ACCEL_AVG_64X = 5,
} ACCEL_UI_AVG;

// Accelerometer filter bandwidth.
typedef enum {
    ACCEL_FILT_BYPASSED = 0,
    ACCEL_FILT_180_HZ = 1,
    ACCEL_FILT_121_HZ = 2,
    ACCEL_FILT_73_HZ = 3,
    ACCEL_FILT_53_HZ = 4,
    ACCEL_FILT_34_HZ = 5,
    ACCEL_FILT_25_HZ = 6,
    ACCEL_FILT_16_HZ = 7,
} ACCEL_UI_FILT_BW;

// Interrupt pin behavior.
typedef enum {
    INT_MODE_PULSED = 0,
    INT_MODE_LATCHED = 1,
} INT_MODE;

// Interrupt pin output type.
typedef enum {
    OPEN_DRAIN = 0,
    PUSH_PULL = 1,
} INT_DRIVE_CIRCUIT;

// Interrupt pin polarity.
typedef enum {
    ACTIVE_LOW = 0,
    ACTIVE_HIGH = 1,
} INT_POLARITY;

// FIFO buffer mode.
typedef enum {
    FIFO_MODE_STREAM = 0,
    FIFO_MODE_STOP_FULL = 1
} FIFO_MODE;

// FIFO bypass state.
typedef enum {
    FIFO_NOT_BYPASSED = 0,
    FIFO_BYPASSED = 1
} FIFO_BYPASS;


// --- APEX Motion Engine Enums ---
typedef enum { DMP_POWER_SAVE_OFF = 0, DMP_POWER_SAVE_ON = 1 } DMP_POWER_SAVE;
typedef enum { DMP_INIT_OFF = 0, DMP_INIT_ON = 1 } DMP_INIT;
typedef enum { NO_MEM_RESET = 0, MEM_RESET = 1 } DMP_MEM_RESET;
typedef enum { SMD_OFF = 0, SMD_ON = 1 } SMD_ENABLE;
typedef enum { FREEFALL_OFF = 0, FREEFALL_ON = 1 } FF_ENABLE;
typedef enum { TILT_DETECT_OFF = 0, TILT_DETECT_ON = 1 } TILT_ENABLE;
typedef enum { PEDOMETER_OFF = 0, PEDOMETER_ON = 1 } PED_ENABLE;
typedef enum { DMP_ODR_25_HZ = 0, DMP_ODR_400_HZ = 1, DMP_ODR_50_HZ = 2, DMP_ODR_100_HZ = 3 } DMP_ODR;
typedef enum { DMP_RUNNING = 0, DMP_IDLE = 1 } DMP_IDLE_STATE;
typedef enum { ACTIVITY_UNKNOWN = 0, ACTIVITY_WALK = 1, ACTIVITY_RUN = 2 } PEDOMETER_ACTIVITY;


// --- Configuration Structures ---

// Groups all main sensor settings for initialization.
typedef struct {
    GYRO_MODE gyro_mode;
    ACCEL_MODE accel_mode;
    IDLE idle;
    ACCEL_LP_CLK_SEL accel_lp_clk_sel;
    GYRO_UI_FS_SEL gyro_ui_fs_sel;
    GYRO_ODR gyro_odr;
    ACCEL_UI_FS_SEL accel_ui_fs_sel;
    ACCEL_ODR accel_odr;
    TEMP_FILT_BW temp_filt_bw;
    GYRO_UI_FILT_BW gyro_ui_filt_bw;
    ACCEL_UI_AVG accel_ui_avg;
    ACCEL_UI_FILT_BW accel_ui_filt_bw;
} icm_42670_t;

// Groups settings for the INT1 and INT2 pins.
typedef struct {
    INT_MODE int2_mode;
    INT_DRIVE_CIRCUIT int2_drive_circuit;
    INT_POLARITY int2_polarity;
    INT_MODE int1_mode;
    INT_DRIVE_CIRCUIT int1_drive_circuit;
    INT_POLARITY int1_polarity;
} icm_42670_int_t;

// Groups settings for the APEX motion engine.
typedef struct {
    DMP_MEM_RESET dmp_mem_reset;
    DMP_INIT dmp_init;
    DMP_POWER_SAVE dmp_power_save;
    DMP_ODR dmp_odr;
    PED_ENABLE pedometer_en;
    TILT_ENABLE tilt_en;
    FF_ENABLE freefall_en;
    SMD_ENABLE smd_en;
} icm_42670_apex_t;

// Groups settings for the FIFO buffer.
typedef struct {
    FIFO_MODE fifo_mode;
    FIFO_BYPASS fifo_bypass;
} icm_42670_fifo_t;


#endif // ICM_42670_TYPES_H