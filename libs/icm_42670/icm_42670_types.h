#ifndef ICM_42670_TYPES
#define ICM_42670_TYPES

typedef struct {
    float gx, gy, gz;  // Gyroscope (dps)
} icm_42670_gyro_data;

typedef struct {
    float ax, ay, az;  // Accelerometer (g)
} icm_42670_accel_data;

typedef struct {
    float ax, ay, az;  // Accelerometer (g)
    float gx, gy, gz;  // Gyroscope (dps)
    int16_t temperature; // Temperature
} icm_42670_all_sensors_data;

typedef enum {
    GYRO_OFF = 0,
    GYRO_STANDBY = 1, 
    GYRO_LN_MODE = 3
} GYRO_MODE;

typedef enum {
    ACCEL_OFF = 0,
    ACCEL_LP_MODE = 2, 
    ACCEL_LN_MODE = 3
} ACCEL_MODE;

typedef enum {
    IDLE_OFF = 0,
    IDLE_ON = 1
} IDLE;

typedef enum {
    WAKE_UP_OSCILLATOR = 0,
    RC_OSCILLATOR = 1
} ACCEL_LP_CLK_SEL;

typedef enum {
    GYRO_UI_2000_DPS = 0,
    GYRO_UI_1000_DPS = 1,
    GYRO_UI_500_DPS = 2,
    GYRO_UI_250_DPS = 3
} GYRO_UI_FS_SEL;

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

typedef enum {
    ACCEL_UI_16_G = 0,
    ACCEL_UI_8_G = 1,
    ACCEL_UI_4_G = 2,
    ACCEL_UI_2_G = 3
} ACCEL_UI_FS_SEL;

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

typedef enum {
    TEMP_FILT_BYPASSED = 0,
    TEMP_FILT_180_HZ = 1,
    TEMP_FILT_72_HZ = 2,
    TEMP_FILT_34_HZ = 3,
    TEMP_FILT_16_HZ = 4,
    TEMP_FILT_8_HZ = 5,
    TEMP_FILT_4_HZ = 6,
} TEMP_FILT_BW;

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

typedef enum {
    ACCEL_AVG_2X = 0,
    ACCEL_AVG_4X = 1,
    ACCEL_AVG_8X = 2,
    ACCEL_AVG_16X = 3,
    ACCEL_AVG_32X = 4,
    ACCEL_AVG_64X = 5,
} ACCEL_UI_AVG;

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

#endif