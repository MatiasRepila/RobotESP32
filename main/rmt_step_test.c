#include "rmt_step_test.h"
#include "esp_check.h"
#include <stdlib.h>

#define TAG "RMTSTEP"

esp_err_t rmt_step_init(rmt_step_t *m, gpio_num_t step, gpio_num_t dir, uint32_t resolution_hz)
{
    ESP_RETURN_ON_FALSE(m, ESP_ERR_INVALID_ARG, TAG, "ptr nulo");
    m->gpio_step = step;
    m->gpio_dir  = dir;
    m->resolution_hz = resolution_hz;

    // DIR como GPIO
    ESP_RETURN_ON_ERROR(gpio_set_direction(dir, GPIO_MODE_OUTPUT), TAG, "gpio dir");
    ESP_RETURN_ON_ERROR(gpio_set_level(dir, 0), TAG, "dir init");

    // Canal TX
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = step,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 64,
        .resolution_hz = resolution_hz,
        .trans_queue_depth = 4,
        .flags.invert_out = false,
        .flags.with_dma = false,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_cfg, &m->chan), TAG, "new_tx");
    ESP_RETURN_ON_ERROR(rmt_enable(m->chan), TAG, "enable");

    // Encoder copy
    rmt_copy_encoder_config_t enc_cfg = {0};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&enc_cfg, &m->enc), TAG, "new_enc");
    return ESP_OK;
}

esp_err_t rmt_step_pulse(rmt_step_t *m, bool dir_fwd, float steps_per_sec, uint32_t dur_ms, uint32_t pulse_us)
{
    ESP_RETURN_ON_FALSE(m, ESP_ERR_INVALID_ARG, TAG, "ptr nulo");
    if (steps_per_sec <= 0 || dur_ms == 0) return ESP_OK;

    // niveles
    ESP_RETURN_ON_ERROR(gpio_set_level(m->gpio_dir, dir_fwd ? 1 : 0), TAG, "set dir");

    // Periodo y ancho en ticks
    const uint32_t period_ticks = (uint32_t)((float)m->resolution_hz / steps_per_sec + 0.5f);
    uint32_t pulse_ticks = (uint32_t)((m->resolution_hz / 1000000.0f) * pulse_us + 0.5f);
    if (pulse_ticks == 0) pulse_ticks = 1;
    if (pulse_ticks >= period_ticks) return ESP_ERR_INVALID_ARG; // pulso no puede ocupar todo el período

    const uint32_t low_ticks = period_ticks - pulse_ticks;

    // Pasos a emitir en la ventana
    uint32_t steps = (uint32_t)(steps_per_sec * (dur_ms / 1000.0f) + 0.5f);
    if (steps == 0) steps = 1;

    // Buffer (símbolo HIGH+LOW por paso)
    const uint32_t CHUNK = 1024;
    rmt_symbol_word_t *buf = malloc(CHUNK * sizeof(*buf));
    ESP_RETURN_ON_FALSE(buf, ESP_ERR_NO_MEM, TAG, "malloc");

    while (steps > 0) {
        uint32_t n = (steps > CHUNK) ? CHUNK : steps;
        for (uint32_t i = 0; i < n; i++) {
            buf[i].level0 = 1;
            buf[i].duration0 = pulse_ticks;
            buf[i].level1 = 0;
            buf[i].duration1 = low_ticks;
        }
        rmt_transmit_config_t tcfg = {.loop_count = 0};
        ESP_RETURN_ON_ERROR(rmt_transmit(m->chan, m->enc, buf, n * sizeof(*buf), &tcfg), TAG, "tx");
        ESP_RETURN_ON_ERROR(rmt_tx_wait_all_done(m->chan, -1), TAG, "wait");
        steps -= n;
    }

    free(buf);
    return ESP_OK;
}
