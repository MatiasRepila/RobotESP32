#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>
#include <stdbool.h>
#include "rmt_step_test.h"   // tu RMT STEP/DIR
#include "control_task.h"    // motor_cmd_t

typedef struct {
    QueueHandle_t    q_in;            // cola desde control_task (motor_cmd_t)
    rmt_step_t      *rmt;             // canal RMT ya inicializado
    uint32_t         period_ms;       // típico 10 ms (100 Hz)
    float            v_max_steps_s;   // saturación de velocidad (p.ej. 10000)
    uint32_t         pulse_us;        // ancho de pulso STEP (us), p.ej. 2
    float            deadband_steps_s;// umbral para no emitir (p.ej. 1.0)
} motor_task_cfg_t;

void motor_task_start(const motor_task_cfg_t *cfg);
