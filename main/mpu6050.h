#pragma once
#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===== Rangos =====
typedef enum { MPU6050_ACC_2G=0, MPU6050_ACC_4G, MPU6050_ACC_8G, MPU6050_ACC_16G } mpu6050_acc_fs_t;
typedef enum { MPU6050_GYR_250=0, MPU6050_GYR_500, MPU6050_GYR_1000, MPU6050_GYR_2000 } mpu6050_gyr_fs_t;

// ===== Handler =====
typedef struct {
    i2c_port_t port;      // I2C_NUM_0 / I2C_NUM_1
    uint8_t addr;         // 0x68 u 0x69 (7-bit, sin shift)
    mpu6050_acc_fs_t acc_fs;
    mpu6050_gyr_fs_t gyr_fs;

    // Estado filtro (solo roll)
    float roll_deg;
    int   first_update;   // para inicializar con ACC la 1ra vez
    int64_t t_us;         // timestamp último update

    // Bias de gyro (deg/s)
    float gx0, gy0, gz0;
} mpu6050_t;

// ===== API =====
// Init completo: wake + DLPF + sample rate + rangos.
// dlpf_cfg: 0..6 (0≈260Hz, 3≈44Hz recomendado para robot)
// rate_hz: típico 100 (SMPLRT_DIV se calcula solo)
esp_err_t mpu6050_init(mpu6050_t *imu,
                       i2c_port_t port, uint8_t addr7,
                       mpu6050_acc_fs_t acc_fs, mpu6050_gyr_fs_t gyr_fs,
                       uint8_t dlpf_cfg, uint16_t rate_hz);

// Ajustes básicos (opcionales)
esp_err_t mpu6050_set_dlpf(mpu6050_t *imu, uint8_t dlpf_cfg);      // 0..6
esp_err_t mpu6050_set_sample_rate(mpu6050_t *imu, uint16_t rate_hz);
esp_err_t mpu6050_set_scales(mpu6050_t *imu, mpu6050_acc_fs_t acc_fs, mpu6050_gyr_fs_t gyr_fs);

// Calibrar bias de gyro en reposo
esp_err_t mpu6050_calibrate_gyro(mpu6050_t *imu, int samples, int ms_between);

// Lecturas escaladas
esp_err_t mpu6050_read_accel(mpu6050_t *imu, float *ax, float *ay, float *az);
esp_err_t mpu6050_read_gyro (mpu6050_t *imu, float *gx, float *gy, float *gz);
esp_err_t mpu6050_read_roll_dps(mpu6050_t *imu, float *roll_dps);

// Paso de filtro complementario (solo roll):
// alpha en [0..1], p.ej. 0.98 a 100 Hz.
// Actualiza imu->roll_deg y opcionalmente lo devuelve en roll_out.
esp_err_t mpu6050_step_complementary_roll(mpu6050_t *imu, float alpha, float *roll_out);




// Getter rápido
static inline float mpu6050_get_roll (const mpu6050_t *imu){ return imu->roll_deg; }

#ifdef __cplusplus
}
#endif