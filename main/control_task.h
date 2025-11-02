#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "control_bus.h"

#ifdef __cplusplus
extern "C" {
#endif

//FSM
typedef enum{
    CTRL_DISABLED = 0, // salida = 0, motores apagados
    CTRL_ARMING   = 1, // dentro de ventana segura, esperando hold-time
    CTRL_ENABLED  = 2, // PD activo
    CTRL_FAULT    = 3  // error: datos inválidos / timing/ etc.
} control_state_t;

// Estado visible desde afuera (para UI/telemetría)
control_state_t control_get_state(void);

// Parámetros: set/get atómicos 
void control_set_params(const control_params_t *p);
void control_get_params(control_params_t *out);

// Arranque de la tarea de control (crea colas que falten y lanza el loop)
void control_task_start(UBaseType_t prio, uint32_t stack_words);



#ifdef __cplusplus
}
#endif