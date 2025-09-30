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

// --- Registros MPU6050 ---
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B
#define REG_GYRO_XOUT_H   0x43
#define REG_PWR_MGMT_1    0x6B


// --- I2C helpers ---
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
// Utilidades para ángulos en grados
static inline float wrap180f(float a_deg) {
    while (a_deg > 180.0f) a_deg -= 360.0f;
    while (a_deg < -180.0f) a_deg += 360.0f;
    return a_deg;
}
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

// --- Sensibilidades ---
static float acc_sens(mpu6050_acc_fs_t fs){
    switch(fs){ case MPU6050_ACC_2G: return 16384.f; case MPU6050_ACC_4G: return 8192.f;
                case MPU6050_ACC_8G: return 4096.f; default: return 2048.f; }
}
static float gyr_sens(mpu6050_gyr_fs_t fs){
    switch(fs){ case MPU6050_GYR_250: return 131.f; case MPU6050_GYR_500: return 65.5f;
                case MPU6050_GYR_1000: return 32.8f; default: return 16.4f; }
}

// --- Config de registros ---
esp_err_t mpu6050_set_dlpf(mpu6050_t *imu, uint8_t dlpf_cfg){
    if (!imu) return ESP_ERR_INVALID_ARG;
    uint8_t v = (dlpf_cfg & 0x07);
    return i2c_wr(imu->port, imu->addr, REG_CONFIG, &v, 1);
}
esp_err_t mpu6050_set_sample_rate(mpu6050_t *imu, uint16_t rate_hz){
    if (!imu || rate_hz == 0) return ESP_ERR_INVALID_ARG;
    // Con DLPF habilitado, Fs interna = 1 kHz
    uint16_t Fs = 1000;
    uint8_t div = (uint8_t)((Fs / rate_hz) - 1);
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

// --- Init ---
esp_err_t mpu6050_init(mpu6050_t *imu,
                       i2c_port_t port, uint8_t addr7,
                       mpu6050_acc_fs_t acc_fs, mpu6050_gyr_fs_t gyr_fs,
                       uint8_t dlpf_cfg, uint16_t rate_hz)
{
    if (!imu) return ESP_ERR_INVALID_ARG;
    imu->port = port;
    imu->addr = addr7;

    // Wake (clear SLEEP)
    uint8_t v=0;
    i2c_rd(port, addr7, REG_PWR_MGMT_1, &v, 1);
    v &= ~(1<<6);
    ESP_RETURN_ON_ERROR(i2c_wr(port, addr7, REG_PWR_MGMT_1, &v, 1), "MPU", "wake");

    // DLPF + rate + escalas
    ESP_RETURN_ON_ERROR(mpu6050_set_dlpf(imu, dlpf_cfg), "MPU", "dlpf");
    ESP_RETURN_ON_ERROR(mpu6050_set_sample_rate(imu, rate_hz), "MPU", "rate");
    ESP_RETURN_ON_ERROR(mpu6050_set_scales(imu, acc_fs, gyr_fs), "MPU", "scales");

    // Estado filtro
    imu->roll_deg = 0.f;
    imu->pitch_deg = 0.f;
    imu->first_update = 1;
    imu->gx0 = imu->gy0 = imu->gz0 = 0.f;
    imu->t_us = esp_timer_get_time();

    return ESP_OK;
}

// --- Lecturas escaladas ---
esp_err_t mpu6050_read_accel(mpu6050_t *imu, float *ax, float *ay, float *az){
    uint8_t b[6];
    ESP_RETURN_ON_ERROR(i2c_rd(imu->port, imu->addr, REG_ACCEL_XOUT_H, b, 6), "MPU", "acc rd");
    int16_t x = (b[0]<<8)|b[1], y = (b[2]<<8)|b[3], z = (b[4]<<8)|b[5];
    float s = acc_sens(imu->acc_fs);
    if (ax) *ax = x/s;
    if (ay) *ay = y/s;
    if (az) *az = z/s;
    return ESP_OK;
}
esp_err_t mpu6050_read_gyro(mpu6050_t *imu, float *gx, float *gy, float *gz){
    uint8_t b[6];
    ESP_RETURN_ON_ERROR(i2c_rd(imu->port, imu->addr, REG_GYRO_XOUT_H, b, 6), "MPU", "gyro rd");
    int16_t x = (b[0]<<8)|b[1], y = (b[2]<<8)|b[3], z = (b[4]<<8)|b[5];
    float s = gyr_sens(imu->gyr_fs);
    float gx_dps = x/s - imu->gx0;
    float gy_dps = y/s - imu->gy0;
    float gz_dps = z/s - imu->gz0;
    if (gx) *gx = gx_dps;
    if (gy) *gy = gy_dps;
    if (gz) *gz = gz_dps;
    return ESP_OK;
}

// --- Calibración gyro ---
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

esp_err_t mpu6050_step_complementary(mpu6050_t *imu, float alpha, mpu6050_angles_t *out)
{
    float ax, ay, az, gx_dps, gy_dps, gz_dps;
    ESP_RETURN_ON_ERROR(mpu6050_read_accel(imu, &ax, &ay, &az), "mpu", "acc read");
    ESP_RETURN_ON_ERROR(mpu6050_read_gyro (imu, &gx_dps, &gy_dps, &gz_dps), "mpu", "gyr read");

    // Remapeos opcionales según orientación física (ejemplos):
    // float ax2 =  ax, ay2 =  ay, az2 =  az;      // sin remapeo
    // float gx2 =  gx_dps, gy2 =  gy_dps, gz2 =  gz_dps;
    // Si tu placa está de costado, por ejemplo: X<-Y, Y<-X:
    // float ax2 =  ay, ay2 =  ax, az2 = az;
    // float gx2 =  gy_dps, gy2 =  gx_dps, gz2 = gz_dps;
    float ax2 = ax, ay2 = ay, az2 = az;
    float gx2 = gx_dps, gy2 = gy_dps, gz2 = gz_dps;

    // Timestamps (us) -> dt (s)
    int64_t t_now = esp_timer_get_time();
    float dt = 0.0f;
    if (imu->t_us == 0) {
        imu->t_us = t_now;
        imu->first_update = 1;
        // inicializamos con ACC más abajo
    } else {
        dt = (float)(t_now - imu->t_us) * 1e-6f;
        imu->t_us = t_now;
        // En caso de jitter muy grande, recortar dt
        dt = clampf(dt, 0.0005f, 0.05f); // 0.5ms..50ms
    }

    // Lecturas gyro en rad/s (bias ya debería estar removido por tu calibración)
    float p = gx2 * DEG2RAD;
    float q = gy2 * DEG2RAD;
    float r = gz2 * DEG2RAD;

    // Ángulos actuales (grados -> rad)
    float phi   = imu->roll_deg  * DEG2RAD;
    float theta = imu->pitch_deg * DEG2RAD;

    // Estimación de ángulos desde acelerómetro (rad -> deg)
    // Robusto cerca de 90°: usar atan2 y sqrt
    float roll_acc_deg  = atan2f(ay2, az2) * RAD2DEG;
    float pitch_acc_deg = atan2f(-ax2, sqrtf(ay2*ay2 + az2*az2)) * RAD2DEG;

    if (imu->first_update) {
        imu->roll_deg  = roll_acc_deg;
        imu->pitch_deg = pitch_acc_deg;
        imu->first_update = 0;
        if (out) { out->roll_deg = imu->roll_deg; out->pitch_deg = imu->pitch_deg; }
        return ESP_OK;
    }

    // === Integración con cinemática Euler (evita errores grandes a 90°) ===
    // Proteger singularidades: cos(theta) no debe ser ~0
    float ct = cosf(theta);
    float st = sinf(theta);
    float sp = sinf(phi);
    float cp = cosf(phi);

    // Limitar tan(theta) de forma segura
    float ct_safe = (fabsf(ct) < 1e-3f) ? (ct >= 0 ? 1e-3f : -1e-3f) : ct;
    float tan_t = st / ct_safe;

    // Derivadas de Euler (rad/s)
    float phi_dot   = p + q*sp*tan_t + r*cp*tan_t;
    float theta_dot =     q*cp        - r*sp;

    // Integración (rad)
    phi   += phi_dot   * dt;
    theta += theta_dot * dt;

    // A grados
    float roll_gyro_deg  = phi   * RAD2DEG;
    float pitch_gyro_deg = theta * RAD2DEG;

    // === Fusión complementaria con "shortest path" (evita salto 180°) ===
    // Asegurar que el error use el camino corto en [-180,180]
    float err_roll  = wrap180f(roll_acc_deg  - roll_gyro_deg);
    float err_pitch = wrap180f(pitch_acc_deg - pitch_gyro_deg);

    float roll_fused  = roll_gyro_deg  + (1.0f - alpha) * err_roll;
    float pitch_fused = pitch_gyro_deg + (1.0f - alpha) * err_pitch;

    // Normalizaciones y límites útiles
    roll_fused  = wrap180f(roll_fused);
    pitch_fused = clampf(pitch_fused, -89.9f, 89.9f); // evita singularidad exacta

    imu->roll_deg  = roll_fused;
    imu->pitch_deg = pitch_fused;

    if (out) {
        out->roll_deg  = imu->roll_deg;
        out->pitch_deg = imu->pitch_deg;
    }
    return ESP_OK;
}
