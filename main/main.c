#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "esp_check.h"
#include "math.h"

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define MPU_ADDR 0x68

// Loop de estimación de ángulo
#define IMU_LOOP_HZ   100
#define IMU_LOOP_MS   (1000 / IMU_LOOP_HZ)
// Filtro complementario: ~0.98 a 100 Hz es un clásico para robots
#define COMP_ALPHA    0.98f

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define RAD2DEG (180.0f / (float)M_PI)



static const char *TAG = "MPU6050:";

// Handler global para que la tarea lo use
static mpu6050_t imu;

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

// ===== Tarea de medición de ángulo (roll/pitch) =====
static void imu_acc_angles_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz
    TickType_t last = xTaskGetTickCount();

    while (1) {
        float ax, ay, az;
        if (mpu6050_read_accel(&imu, &ax, &ay, &az) == ESP_OK) {
            // Normalización opcional (no estrictamente necesaria si ya está en "g")
            float n = sqrtf(ax*ax + ay*ay + az*az);
            if (n < 1e-6f) n = 1e-6f;
            float axu = ax / n, ayu = ay / n, azu = az / n;

            // Ángulos desde ACC (convención clásica, Z hacia arriba en reposo)
            // roll  = rotación alrededor de X
            // pitch = rotación alrededor de Y
            float roll_deg  = atan2f(ayu, azu) * RAD2DEG;
            float pitch_deg = atan2f(-axu, sqrtf(ayu*ayu + azu*azu)) * RAD2DEG;

            // Log crudo + ángulos
            ESP_LOGI(TAG, "ACC[g]: ax=%+6.3f ay=%+6.3f az=%+6.3f | roll=%+7.3f° pitch=%+7.3f°",
                     ax, ay, az, roll_deg, pitch_deg);
        } else {
            ESP_LOGW(TAG, "Fallo lectura ACC");
        }

        vTaskDelayUntil(&last, period);
    }
}

// ===== app_main =====
void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C inicializado");

    ESP_ERROR_CHECK(
        mpu6050_init(&imu,
                     I2C_MASTER_NUM,
                     MPU_ADDR,
                     MPU6050_ACC_2G,
                     MPU6050_GYR_250,
                     3,           // DLPF=3 (~44 Hz)
                     IMU_LOOP_HZ  // Fs=100 Hz
        )
    );
    ESP_LOGI(TAG, "MPU listo: ACC=±2g, GYR=±250dps, DLPF=3, Fs=%d Hz", IMU_LOOP_HZ);

    // Dejar que se estabilice el sensor un instante
    vTaskDelay(pdMS_TO_TICKS(200));

    // Calibración de bias del giroscopio (mantener inmóvil)
    ESP_LOGI(TAG, "Calibrando giroscopio (0.5 s aprox), no mover...");
    ESP_ERROR_CHECK(mpu6050_calibrate_gyro(&imu, /*samples*/100, /*ms_between*/5));
    ESP_LOGI(TAG, "Calibración OK: gx0=%.3f, gy0=%.3f, gz0=%.3f (deg/s)",
             imu.gx0, imu.gy0, imu.gz0);

    xTaskCreate(imu_acc_angles_task, "imu_acc_angles_task", 3*1024, NULL,
                tskIDLE_PRIORITY + 2, NULL);

    // Nada más que hacer en main; las tareas corren solas
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
