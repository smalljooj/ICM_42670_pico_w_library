#include <stdio.h>
#include <math.h>
#include "icm_42670.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"

const float ALPHA = 0.98f;
const float PI = 3.14159265359f;

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
#define RESET_PIN 20
#define BOOT_PIN 26
#define GND_PIN 14 

void gpio_callback(uint gpio, uint32_t events);

void configure_boot_pin();

int main()
{
    int16_t ax, ay, az, gx, gy, gz; 

    stdio_init_all();
    configure_boot_pin();

    init_spi();
    sleep_ms(1000);

    icm_42670_status();
    icm_42670_t icm_init = {
        .gyro_mode = GYRO_LN_MODE,
        .accel_mode = ACCEL_LN_MODE,
        .idle = IDLE_OFF,
        .accel_lp_clk_sel = WAKE_UP_OSCILLATOR,
        .gyro_ui_fs_sel = GYRO_UI_2000_DPS,
        .gyro_odr = GYRO_ODR_1600HZ,
        .accel_ui_fs_sel = ACCEL_UI_16_G,
        .accel_odr = ACCEL_ODR_1600HZ,
        .temp_filt_bw = TEMP_FILT_BYPASSED,
        .gyro_ui_filt_bw = GYRO_FILT_BYPASSED,
        .accel_ui_avg = ACCEL_AVG_32X,
        .accel_ui_filt_bw = ACCEL_FILT_BYPASSED,
    };
    init_icm_42670(&icm_init);

    icm_42670_status();

    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    icm_42670_all_sensors_data_t data;
    icm_42670_angles_data_t angles;

    uint64_t last_time = time_us_64();


    while (true) {
        uint64_t current_time = time_us_64();

        icm_42670_read_all_sensors(&data);
        float ax_g = data.ax / 2048.0f;
        float ay_g = data.ay / 2048.0f;
        float az_g = data.az / 2048.0f;

        float gx_dps = data.gx / 16.4f;
        float gy_dps = data.gy / 16.4f;
        float gz_dps = data.gz / 16.4f;

        double pitch, roll;

        icm_42670_kalman_update();

        if (current_time - last_time >= 1000000) {
            last_time = current_time;
            icm_42670_kalman_get_angles(&angles);
            printf("Accel: [%.2fg %.2fg %.2fg] | Gyro: [%.2f %.2f %.2f] dps\n", ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);
            printf("pitch: %f  -   roll: %f\n\n", angles.pitch, angles.roll);
        }
    }
}

void gpio_callback(uint gpio, uint32_t events) {
    if(gpio == RESET_PIN) {
        watchdog_reboot(0, 0, 0);  // Reset normal
    } else if(gpio == BOOT_PIN) {
        reset_usb_boot(0, 0);      // Entra no modo bootloader
    }
}

void configure_boot_pin()
{
    gpio_init(GND_PIN);
    gpio_set_dir(GND_PIN, GPIO_OUT);
    // Configuração dos pinos
    gpio_init(RESET_PIN);
    gpio_set_dir(RESET_PIN, GPIO_IN);
    gpio_pull_up(RESET_PIN);
    
    gpio_init(BOOT_PIN);
    gpio_set_dir(BOOT_PIN, GPIO_IN);
    gpio_pull_up(BOOT_PIN);
    
    // Configurar interrupções
    gpio_set_irq_enabled_with_callback(RESET_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(BOOT_PIN, GPIO_IRQ_EDGE_FALL, true);
}