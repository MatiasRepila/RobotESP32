#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050.h"


#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define MPU6050_ADDR                0x68      // Cambia a 0x69 si AD0 está en alto
#define MPU6050_REG_WHO_AM_I        0x75
#define MPU6050_REG_PWR_MGMT_1      0x6B

static const char *TAG = "MPU6050_WHOAMI";

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

// --- Helpers de lectura/escritura de registros ---
static esp_err_t mpu6050_write_byte(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR,
                                      buf, sizeof(buf), pdMS_TO_TICKS(50));
}

static esp_err_t mpu6050_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    // Write del registro + Read de 'len' bytes
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR,
                                        &reg, 1, data, len, pdMS_TO_TICKS(50));
}

// --- Wake + WHO_AM_I ---
void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C inicializado");

    // El MPU6050 arranca en sleep; ponemos PWR_MGMT_1 = 0x00 (usa reloj interno)
    ESP_ERROR_CHECK(mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, 0x00));
    vTaskDelay(pdMS_TO_TICKS(10));  // pequeño settle

    // Leer WHO_AM_I
    uint8_t who = 0;
    ESP_ERROR_CHECK(mpu6050_read_bytes(MPU6050_REG_WHO_AM_I, &who, 1));
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (esperado 0x68)", who);

    if (who == 0x68) {
        ESP_LOGI(TAG, "MPU6050 detectado OK en 0x%02X", MPU6050_ADDR);
    } else {
        ESP_LOGE(TAG, "ID inesperado. Revisá AD0 (0x69?) o el bus.");
    }

    // (Opcional) Loop vacío: dejamos el task vivo
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
