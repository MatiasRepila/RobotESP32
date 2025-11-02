#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// --- Mensaje IMU -> Control ---
typedef struct {
    float roll_deg;
    float roll_dps;
    float dt;
    TickType_t tick;
} imu_msg_t;

extern QueueHandle_t q_imu2ctrl;

// --- Parámetros del control (mover acá para evitar includes cruzados) ---
typedef struct {
    float kp_int;
    float kd_int;
    float ref_deg;
    float theta_enable_deg;
    float theta_disable_deg;
    float dps_enable_max;
    uint32_t hold_enable_ms;
    float out_limit_abs;
    float out_slew_max;
} control_params_t;

// --- Comandos hacia control ---
typedef enum {
    CTRL_CMD_SET_PARAMS = 0,
} control_cmd_type_t;

typedef struct {
    control_cmd_type_t type;
    control_params_t   params;
} control_cmd_t;

extern QueueHandle_t q_ctrl_cmd;