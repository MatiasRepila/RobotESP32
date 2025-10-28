#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

// Mensaje que consumirá la tarea del motor
typedef struct {
    float accel_steps_s2;   // aceleración deseada [steps/s^2]
} motor_cmd_t;

// Configuración para iniciar la tarea de control
typedef struct {
    QueueHandle_t q_out;    // cola hacia motor_task (obligatoria)
    uint32_t      period_ms;// típico: 10 ms (100 Hz)
    float         A_steps_s2;// amplitud de aceleración para prueba (p.ej. 3000)
} control_task_cfg_t;

// Crea/lanza la tarea de control (no bloquea)
void control_task_start(const control_task_cfg_t *cfg);
