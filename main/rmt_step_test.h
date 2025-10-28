#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
    gpio_num_t gpio_step;
    gpio_num_t gpio_dir;
    uint32_t   resolution_hz;   // p.ej. 1 MHz => 1 tick = 1 us
    rmt_channel_handle_t chan;
    rmt_encoder_handle_t enc;
} rmt_step_t;

esp_err_t rmt_step_init(rmt_step_t *m, gpio_num_t step, gpio_num_t dir, uint32_t resolution_hz);

// genera pasos a 'steps_per_sec' durante 'dur_ms' con pulso alto mínimo 'pulse_us'
esp_err_t rmt_step_pulse(rmt_step_t *m, bool dir_fwd, float steps_per_sec, uint32_t dur_ms, uint32_t pulse_us);
