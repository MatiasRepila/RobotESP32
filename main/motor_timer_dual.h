#pragma once
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"


#ifdef __cplusplus
extern "C" {
#endif

// Arranca generador STEP/DIR dual con GPTimer (ISR periódica estilo Arduino).
// isr_hz: frecuencia de interrupción del timer (p.ej. 50000 = 20 us por tick)
// pulse_ticks: ancho del pulso STEP en ticks de ISR (2 → ~40 us a 50 kHz).
void motor_timer_dual_start(gpio_num_t step_L, gpio_num_t dir_L,
                            gpio_num_t step_R, gpio_num_t dir_R,
                            uint32_t isr_hz, uint32_t pulse_ticks);

// Setea consignas por motor (freq en steps/s >= 0, dir 0/1).
void motor_timer_set_L(float freq_steps_s, int dir_level);
void motor_timer_set_R(float freq_steps_s, int dir_level);

// Task ligera que integra u_pd -> vel y alimenta ambos motores (100 Hz).
void motor_task_timer_start(UBaseType_t prio, uint32_t stack_words);

// Tuning de límites (usados por la task).
void motor_set_k_accel(float k_steps_s2_per_u);
void motor_set_limits(float vmax_steps_s, float amax_steps_s2);

#ifdef __cplusplus
}
#endif
