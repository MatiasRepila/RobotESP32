#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"     // <-- agregado
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_check.h"
#include "mpu6050.h"

#include "rmt_step_test.h"      // RMT STEP/DIR
#include "control_task.h"       // aceleración ficticia (steps/s^2)
#include "motor_task.h"         // integra a->v y llama a RMT   <-- agregado

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
#define COMP_ALPHA    0.98f

// ===== STEP/DIR RMT =====
#define GPIO_STEP        ((gpio_num_t)18)
#define GPIO_DIR         ((gpio_num_t)19)
#define RMT_RES_HZ       (1*1000*1000)   // 1 MHz => 1 tick = 1 us
#define STEP_PULSE_US    2               // ancho HIGH del STEP (ajustar a tu driver)
#define V_MAX_STEPS_S    10000.0f        // saturación de velocidad
#define DEAD_BAND_SPS    1.0f            // umbral para no emitir (evita 1 paso suelto)
#define CTRL_PERIOD_MS   10              // 100 Hz
#define MOTOR_PERIOD_MS  10              // 100 Hz (igual que control)

static const char *TAG = "APP";

// ==== Handlers ====
static mpu6050_t imu;
static rmt_step_t s_step;
static QueueHandle_t g_q_ctrl_to_motor;

// ==== Init I2C (opcional) ====
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

#if 0
// ===== Tarea IMU (opcional) =====
static void imu_complementary_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(IMU_LOOP_MS);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        float roll_deg = 0.0f;
        esp_err_t err = mpu6050_step_complementary_roll(&imu, COMP_ALPHA, &roll_deg);
        if (err == ESP_OK) {
            ESP_LOGI("MPU6050", "ROLL= %+7.3f°", roll_deg);
        }
        vTaskDelayUntil(&last, period);
    }
}
#endif

// ===== app_main =====
void app_main(void)
{
    // --- (Opcional) IMU: dejada comentada para centrar pruebas en motores ---
#if 0
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C inicializado");

    ESP_ERROR_CHECK(
        mpu6050_init(&imu,
                     I2C_MASTER_NUM,
                     MPU_ADDR,
                     MPU6050_ACC_2G,
                     MPU6050_GYR_250,
                     3,            // DLPF=3 (~44 Hz)
                     IMU_LOOP_HZ)  // Fs=100 Hz
    );
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_ERROR_CHECK(mpu6050_calibrate_gyro(&imu, /*samples*/100, /*ms_between*/5));
    xTaskCreate(imu_complementary_task, "imu_complementary_task",
                3*1024, NULL, tskIDLE_PRIORITY + 1, NULL);
#endif

    
    // 1) Cola control -> motor (último comando de aceleración)
    g_q_ctrl_to_motor = xQueueCreate(1, sizeof(motor_cmd_t));
    configASSERT(g_q_ctrl_to_motor);

    // 2) RMT STEP/DIR
    ESP_ERROR_CHECK(rmt_step_init(&s_step, GPIO_STEP, GPIO_DIR, RMT_RES_HZ));

    // 3) Tarea de CONTROL (100 Hz) con aceleración ficticia
    control_task_cfg_t ctrl_cfg = {
        .q_out       = g_q_ctrl_to_motor,
        .period_ms   = CTRL_PERIOD_MS,
        .A_steps_s2  = 3000.0f
    };
    control_task_start(&ctrl_cfg);

    // 4) Tarea de MOTOR (100 Hz): integra a->v, satura y emite con RMT
    motor_task_cfg_t mot_cfg = {
        .q_in             = g_q_ctrl_to_motor,
        .rmt              = &s_step,
        .period_ms        = MOTOR_PERIOD_MS,
        .v_max_steps_s    = V_MAX_STEPS_S,
        .pulse_us         = STEP_PULSE_US,
        .deadband_steps_s = DEAD_BAND_SPS
    };
    motor_task_start(&mot_cfg);

    // 5) Idle
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
