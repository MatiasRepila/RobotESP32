#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "esp_check.h"
#include "control_bus.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include <math.h>
#include "control_task.h"
#include "bt_spp.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"



// ===== Pines y configuración I2C =====
#define I2C_MASTER_SCL_IO           27
#define I2C_MASTER_SDA_IO           26
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

// Handler global del imu
static mpu6050_t imu;

// Definiciones únicas de las colas 
QueueHandle_t q_imu2ctrl = NULL;
QueueHandle_t q_ctrl_cmd = NULL;


//Init I2C 
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

//Tarea: filtro complementario 
static void imu_complementary_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(IMU_LOOP_MS);
    TickType_t last = xTaskGetTickCount();
    const float dt = 1.0f / (float)IMU_LOOP_HZ;

    for (;;) {
        float roll_deg = 0.0f;
        if (mpu6050_step_complementary_roll(&imu, COMP_ALPHA, &roll_deg) == ESP_OK) {

            float roll_dps = 0.0f;
            if (mpu6050_read_roll_dps(&imu, &roll_dps) != ESP_OK) {
                // Si fallara, mantenemos el último valor o 0
                roll_dps = 0.0f;
            }

            imu_msg_t msg = {
                .roll_deg = roll_deg,
                .roll_dps = roll_dps,
                .dt       = dt,
                .tick     = xTaskGetTickCount()
            };

            // Se usa overwrite porque la cola tiene longitud uno, entonces siempre tenemos el dato fresco
            if (q_imu2ctrl) {
                xQueueOverwrite(q_imu2ctrl, &msg);
                
            }
            //ESP_LOGI("imu", "pub roll=%.2f w=%.2f", msg.roll_deg, msg.roll_dps);
 
            // ESP_LOGI(TAG, "ROLL=%+7.3f°, dROLL=%+7.3f°/s", roll_deg, roll_dps);
        } else {
            ESP_LOGW(TAG, "Fallo step complementario");
        }

        vTaskDelayUntil(&last, period);
    }
}

void app_main(void)
{

    esp_log_level_set("*", ESP_LOG_INFO);      // opcional
    esp_log_level_set("control", ESP_LOG_INFO);

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
    ESP_ERROR_CHECK(mpu6050_calibrate_gyro(&imu, /*samples*/100, /*ms_between*/10));
    ESP_LOGI(TAG, "Bias: gx0=%.3f gy0=%.3f gz0=%.3f (deg/s)", imu.gx0, imu.gy0, imu.gz0);

    ESP_LOGI("app", "Creando colas...");

    q_imu2ctrl = xQueueCreate(1, sizeof(imu_msg_t));
    q_ctrl_cmd = xQueueCreate(1, sizeof(control_cmd_t));
    ESP_ERROR_CHECK(q_imu2ctrl && q_ctrl_cmd ? ESP_OK : ESP_FAIL);

    control_task_start(6, 4906);

    

    // Tarea de estimación de roll con filtro complementario
    xTaskCreate(imu_complementary_task, "imu_complementary_task",
                3*1024, NULL, tskIDLE_PRIORITY + 2, NULL);

    // NVS (antes de cualquier cosa de BT)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // SOLO esto: todo lo demás ocurre adentro de bt_spp_start()
    ESP_ERROR_CHECK(bt_spp_start("RobotBalancin"));
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}