#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_check.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define RAD2DEG (180.0f / (float)M_PI)
#define DEG2RAD ((float)M_PI / 180.0f)

// ===================== CONFIGURABLES (build-time) =====================

// Frecuencia real del loop de tu tarea (Hz). Usada para dt fijo:
#ifndef IMU_RATE_HZ
#define IMU_RATE_HZ 100
#endif
#define IMU_DT (1.0f / (float)IMU_RATE_HZ)

// Rango de confianza del ACC (en unidades "g"); fuera de esto se ignora corrección:
#ifndef ACC_MIN_G
#define ACC_MIN_G 0.70f
#endif
#ifndef ACC_MAX_G
#define ACC_MAX_G 1.30f
#endif

// Alpha base (cuando ACC es poco confiable) y alpha bajo (cuando ACC es confiable)
// Recuerda: alpha alto = confío más en gyro; (1-alpha) es el peso del ACC
#ifndef ALPHA_HIGH
#define ALPHA_HIGH 0.98f
#endif
#ifndef ALPHA_LOW
#define ALPHA_LOW  0.90f
#endif

// Pendiente para mapear desviación |‖a‖-1| -> factor [0..1] (heurístico):
#ifndef ALPHA_DEV_GAIN
#define ALPHA_DEV_GAIN 4.0f
#endif

// Promedio de ACC al inicio para nivelar (0 = deshabilitado)
#ifndef ACC_BOOT_AVG_SAMPLES
#define ACC_BOOT_AVG_SAMPLES 0   // p.ej. 80 para ~0.8 s a 100 Hz
#endif

// ===================== Remapeo de ejes (según montaje físico) =====================
// Elegí qué eje de gyro integrás para ROLL y qué ejes del ACC usás en atan2(Y, Z).
// Por defecto: roll = alrededor de X => uso gx, y para ACC uso (ay, az).
// Si tu roll real fuera alrededor de Y, por ejemplo, podrías usar: GYRO_ROLL gy_dps y (ax, az).

#define GYRO_ROLL(gx_dps,gy_dps,gz_dps)   (gx_dps)   // opciones: (gx_dps) / (gy_dps) / (gz_dps)
#define ACC_Y_FOR_ROLL(ax,ay,az)          (ay)       // opciones: (ax) / (ay) / (az)
#define ACC_Z_FOR_ROLL(ax,ay,az)          (az)       // opciones: (ax) / (ay) / (az)

// ===================== Registros MPU6050 =====================
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B
#define REG_GYRO_XOUT_H   0x43
#define REG_PWR_MGMT_1    0x6B

// ===================== Helpers I2C =====================
static inline esp_err_t i2c_wr(i2c_port_t p, uint8_t a7, uint8_t reg, const uint8_t *buf, size_t n){
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (a7<<1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(c, reg, true);
    if (n) i2c_master_write(c, (uint8_t*)buf, n, true);
    i2c_master_stop(c);
    esp_err_t r = i2c_master_cmd_begin(p, c, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(c);
    return r;
}
static inline esp_err_t i2c_rd(i2c_port_t p, uint8_t a7, uint8_t reg, uint8_t *buf, size_t n){
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (a7<<1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(c, reg, true);
    i2c_master_start(c);
    i2c_master_write_byte(c, (a7<<1) | I2C_MASTER_READ, true);
    i2c_master_read(c, buf, n, I2C_MASTER_LAST_NACK);
    i2c_master_stop(c);
    esp_err_t r = i2c_master_cmd_begin(p, c, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(c);
    return r;
}

// ===================== Utils =====================
static inline float wrap180f(float a_deg) {
    while (a_deg > 180.0f) a_deg -= 360.0f;
    while (a_deg < -180.0f) a_deg += 360.0f;
    return a_deg;
}
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

// ===================== Sensibilidades =====================
static float acc_sens(mpu6050_acc_fs_t fs){
    switch(fs){ case MPU6050_ACC_2G: return 16384.f; case MPU6050_ACC_4G: return 8192.f;
                case MPU6050_ACC_8G: return 4096.f; default: return 2048.f; }
}
static float gyr_sens(mpu6050_gyr_fs_t fs){
    switch(fs){ case MPU6050_GYR_250: return 131.f; case MPU6050_GYR_500: return 65.5f;
                case MPU6050_GYR_1000: return 32.8f; default: return 16.4f; }
}

// ===================== Config de registros =====================
esp_err_t mpu6050_set_dlpf(mpu6050_t *imu, uint8_t dlpf_cfg){
    if (!imu) return ESP_ERR_INVALID_ARG;
    uint8_t v = (dlpf_cfg & 0x07);
    return i2c_wr(imu->port, imu->addr, REG_CONFIG, &v, 1);
}
esp_err_t mpu6050_set_sample_rate(mpu6050_t *imu, uint16_t rate_hz){
    if (!imu || rate_hz == 0) return ESP_ERR_INVALID_ARG;
    const uint16_t Fs = 1000;               // con DLPF, Fs interna = 1 kHz
    if (rate_hz > Fs) rate_hz = Fs;         // máx 1000 Hz
    uint32_t div32 = (Fs / rate_hz) - 1;    // 0..255
    if (div32 > 255) div32 = 255;
    uint8_t div = (uint8_t)div32;
    return i2c_wr(imu->port, imu->addr, REG_SMPLRT_DIV, &div, 1);
}
esp_err_t mpu6050_set_scales(mpu6050_t *imu, mpu6050_acc_fs_t acc_fs, mpu6050_gyr_fs_t gyr_fs){
    if (!imu) return ESP_ERR_INVALID_ARG;
    uint8_t v;
    v = ((uint8_t)gyr_fs) << 3;
    ESP_RETURN_ON_ERROR(i2c_wr(imu->port, imu->addr, REG_GYRO_CONFIG, &v, 1), "MPU", "gyro fs");
    v = ((uint8_t)acc_fs) << 3;
    ESP_RETURN_ON_ERROR(i2c_wr(imu->port, imu->addr, REG_ACCEL_CONFIG, &v, 1), "MPU", "acc fs");
    imu->acc_fs = acc_fs;
    imu->gyr_fs = gyr_fs;
    return ESP_OK;
}

// ===================== Init =====================
esp_err_t mpu6050_init(mpu6050_t *imu,
                       i2c_port_t port, uint8_t addr7,
                       mpu6050_acc_fs_t acc_fs, mpu6050_gyr_fs_t gyr_fs,
                       uint8_t dlpf_cfg, uint16_t rate_hz)
{
    if (!imu) return ESP_ERR_INVALID_ARG;
    imu->port = port;
    imu->addr = addr7;

    // Wake (clear SLEEP, reloj interno). Escribir 0x00 es suficiente.
    uint8_t v = 0x00;
    ESP_RETURN_ON_ERROR(i2c_wr(port, addr7, REG_PWR_MGMT_1, &v, 1), "MPU", "wake");

    // DLPF + rate + escalas
    ESP_RETURN_ON_ERROR(mpu6050_set_dlpf(imu, dlpf_cfg), "MPU", "dlpf");
    ESP_RETURN_ON_ERROR(mpu6050_set_sample_rate(imu, rate_hz), "MPU", "rate");
    ESP_RETURN_ON_ERROR(mpu6050_set_scales(imu, acc_fs, gyr_fs), "MPU", "scales");

    // Estado filtro
    imu->roll_deg = 0.f;
    imu->first_update = 1;
    imu->gx0 = imu->gy0 = imu->gz0 = 0.f;
    imu->t_us = esp_timer_get_time();

#if ACC_BOOT_AVG_SAMPLES > 0
    // Promedio de ACC al arranque para “nivelar”
    float axm=0, aym=0, azm=0;
    for (int i = 0; i < ACC_BOOT_AVG_SAMPLES; i++){
        float ax, ay, az;
        if (mpu6050_read_accel(imu, &ax, &ay, &az) == ESP_OK) {
            axm += ax; aym += ay; azm += az;
        }
        vTaskDelay(pdMS_TO_TICKS((int)(1000.0f/IMU_RATE_HZ)));
    }
    if (ACC_BOOT_AVG_SAMPLES > 0){
        axm /= ACC_BOOT_AVG_SAMPLES;
        aym /= ACC_BOOT_AVG_SAMPLES;
        azm /= ACC_BOOT_AVG_SAMPLES;
        float ay2 = ACC_Y_FOR_ROLL(axm,aym,azm);
        float az2 = ACC_Z_FOR_ROLL(axm,aym,azm);
        imu->roll_deg = atan2f(ay2, az2) * RAD2DEG;
        imu->first_update = 0;
    }
#endif

    return ESP_OK;
}

// ===================== Lecturas escaladas =====================
esp_err_t mpu6050_read_accel(mpu6050_t *imu, float *ax, float *ay, float *az){
    if (!imu) return ESP_ERR_INVALID_ARG;
    uint8_t b[6];
    ESP_RETURN_ON_ERROR(i2c_rd(imu->port, imu->addr, REG_ACCEL_XOUT_H, b, 6), "MPU", "acc rd");
    int16_t x = (int16_t)((b[0]<<8)|b[1]), y = (int16_t)((b[2]<<8)|b[3]), z = (int16_t)((b[4]<<8)|b[5]);
    float s = acc_sens(imu->acc_fs);
    if (ax) *ax = x/s;
    if (ay) *ay = y/s;
    if (az) *az = z/s;
    return ESP_OK;
}
esp_err_t mpu6050_read_gyro(mpu6050_t *imu, float *gx, float *gy, float *gz){
    if (!imu) return ESP_ERR_INVALID_ARG;
    uint8_t b[6];
    ESP_RETURN_ON_ERROR(i2c_rd(imu->port, imu->addr, REG_GYRO_XOUT_H, b, 6), "MPU", "gyro rd");
    int16_t x = (int16_t)((b[0]<<8)|b[1]), y = (int16_t)((b[2]<<8)|b[3]), z = (int16_t)((b[4]<<8)|b[5]);
    float s = gyr_sens(imu->gyr_fs);
    float gx_dps = x/s - imu->gx0;
    float gy_dps = y/s - imu->gy0;
    float gz_dps = z/s - imu->gz0;
    if (gx) *gx = gx_dps;
    if (gy) *gy = gy_dps;
    if (gz) *gz = gz_dps;
    return ESP_OK;
}

// ===================== Calibración gyro =====================
esp_err_t mpu6050_calibrate_gyro(mpu6050_t *imu, int samples, int ms_between){
    if (!imu || samples<=0) return ESP_ERR_INVALID_ARG;
    float sx=0, sy=0, sz=0;
    for (int i=0;i<samples;i++){
        float gx,gy,gz;
        ESP_RETURN_ON_ERROR(mpu6050_read_gyro(imu, &gx,&gy,&gz), "MPU", "gyro cal");
        sx += gx; sy += gy; sz += gz;
        vTaskDelay(pdMS_TO_TICKS(ms_between));
    }
    imu->gx0 = sx/samples;
    imu->gy0 = sy/samples;
    imu->gz0 = sz/samples;
    return ESP_OK;
}
// ===================== Lectura Velocidad angular =====================
esp_err_t mpu6050_read_roll_dps(mpu6050_t *imu, float *roll_dps){
    if (!imu || !roll_dps) return ESP_ERR_INVALID_ARG;
    float gx_dps, gy_dps, gz_dps;
    ESP_RETURN_ON_ERROR(mpu6050_read_gyro(imu, &gx_dps, &gy_dps, &gz_dps), "MPU", "gyr rd");
    // Usa el mismo remapeo que el filtro complementario:
    float g_roll = GYRO_ROLL(gx_dps, gy_dps, gz_dps);
    *roll_dps = g_roll;
    return ESP_OK;
}
// ===================== Filtro complementario (solo roll) =====================
esp_err_t mpu6050_step_complementary_roll(mpu6050_t *imu, float alpha_cfg, float *roll_out)
{
    if (!imu) return ESP_ERR_INVALID_ARG;

    float ax, ay, az, gx_dps, gy_dps, gz_dps;
    ESP_RETURN_ON_ERROR(mpu6050_read_accel(imu, &ax, &ay, &az), "MPU", "acc read");
    ESP_RETURN_ON_ERROR(mpu6050_read_gyro (imu, &gx_dps, &gy_dps, &gz_dps), "MPU", "gyr read");

    // Remapeo según montaje físico:
    float ay2 = ACC_Y_FOR_ROLL(ax,ay,az);
    float az2 = ACC_Z_FOR_ROLL(ax,ay,az);
    float g_roll_dps = GYRO_ROLL(gx_dps, gy_dps, gz_dps);

    // Ángulo desde ACC (roll)
    float roll_acc_deg = atan2f(ay2, az2) * RAD2DEG;

    // Primera muestra sin historial -> inícialo con ACC
    if (imu->first_update) {
        imu->roll_deg = roll_acc_deg;
        imu->first_update = 0;
        if (roll_out) *roll_out = imu->roll_deg;
        return ESP_OK;
    }

    // Integración de gyro con dt fijo
    float roll_gyro_deg = imu->roll_deg + g_roll_dps * IMU_DT;

    // Norma del ACC para heurísticas
    float gnorm = sqrtf(ax*ax + ay*ay + az*az);
    bool acc_confiable = (gnorm > ACC_MIN_G && gnorm < ACC_MAX_G);

    // Alpha adaptativo en función de la desviación respecto a 1 g
    float dev = fabsf(gnorm - 1.0f);
    float k = clampf(1.0f - ALPHA_DEV_GAIN * dev, 0.0f, 1.0f); // k≈1: ACC confiable
    float alpha_eff = ALPHA_LOW * k + ALPHA_HIGH * (1.0f - k);

    // Si el ACC está fuera de rango, ignoro su corrección esta iteración
    float err = acc_confiable ? wrap180f(roll_acc_deg - roll_gyro_deg) : 0.0f;

    imu->roll_deg = roll_gyro_deg + (1.0f - alpha_eff) * err;   

    if (roll_out) *roll_out = imu->roll_deg;
    return ESP_OK;
}