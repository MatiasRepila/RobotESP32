#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "esp_check.h"
#include <math.h>

// ===== Pines y configuración I2C =====
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define MPU_ADDR 0x68

// ===== Loop IMU / Filtro =====
#define IMU_LOOP_HZ   100
#define IMU_LOOP_MS   (1000 / IMU_LOOP_HZ)
#define COMP_ALPHA    0.98f   // confianza alta en gyro a 100 Hz

static const char *TAG = "MPU6050";

// Handler global
static mpu6050_t imu;

// --- Init I2C ---
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

// ===== Tarea: filtro complementario (solo roll) =====
static void imu_complementary_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(IMU_LOOP_MS);
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        float roll_deg = 0.0f;
        esp_err_t err = mpu6050_step_complementary_roll(&imu, COMP_ALPHA, &roll_deg);
        if (err == ESP_OK) {
            // roll_deg: ángulo en grados, en [-180, 180]
            ESP_LOGI(TAG, "ROLL= %+7.3f°", roll_deg);
        } else {
            ESP_LOGW(TAG, "Fallo step complementario (%d)", (int)err);
        }
        vTaskDelayUntil(&last, period);
    }
}

// ===== app_main =====
void app_main(void)
{
    // I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C inicializado");

    // IMU
    ESP_ERROR_CHECK(
        mpu6050_init(&imu,
                     I2C_MASTER_NUM,
                     MPU_ADDR,
                     MPU6050_ACC_2G,
                     MPU6050_GYR_250,
                     3,            // DLPF=3 (~44 Hz)
                     IMU_LOOP_HZ)  // Fs=100 Hz
    );
    ESP_LOGI(TAG, "MPU listo: ACC=±2g, GYR=±250dps, DLPF=3, Fs=%d Hz", IMU_LOOP_HZ);

    // Estabilizar sensor
    vTaskDelay(pdMS_TO_TICKS(200));

    // Calibración de bias del gyro (quieto)
    ESP_LOGI(TAG, "Calibrando giroscopio (0.5 s), no mover...");
    ESP_ERROR_CHECK(mpu6050_calibrate_gyro(&imu, /*samples*/100, /*ms_between*/5));
    ESP_LOGI(TAG, "Bias: gx0=%.3f gy0=%.3f gz0=%.3f (deg/s)", imu.gx0, imu.gy0, imu.gz0);

    // Tarea de estimación de roll con filtro complementario
    xTaskCreate(imu_complementary_task, "imu_complementary_task",
                3*1024, NULL, tskIDLE_PRIORITY + 2, NULL);

    // Idle loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
