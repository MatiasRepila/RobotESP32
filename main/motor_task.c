#include "motor_task.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "MOTOR";

typedef struct {
    QueueHandle_t q_in;
    rmt_step_t   *rmt;
    TickType_t    period_ticks;
    float         dt_s;
    float         v_max;
    uint32_t      pulse_us;
    float         deadband;
} motor_rt_t;

static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static void motor_task(void *arg)
{
    motor_rt_t cfg = *(motor_rt_t*)arg;
    vPortFree(arg);

    motor_cmd_t cmd = { .accel_steps_s2 = 0.0f };
    float v_steps = 0.0f;  // estado interno: velocidad actual [steps/s]

    TickType_t last = xTaskGetTickCount();

    for (;;) {
        // Leer último comando disponible (no bloqueante). Si hay varios, me quedo con el más nuevo.
        motor_cmd_t tmp;
        while (xQueueReceive(cfg.q_in, &tmp, 0) == pdTRUE) { cmd = tmp; }

        // Integración: v <- v + a*dt
        v_steps += cmd.accel_steps_s2 * cfg.dt_s;

        // Saturación
        v_steps = clampf(v_steps, -cfg.v_max, cfg.v_max);

        // Emisión por ventana (=periodo de tarea)
        float v = v_steps;
        if (fabsf(v) > cfg.deadband) {
            bool dir = (v >= 0.0f);
            float freq = fabsf(v); // steps/s
            // Ventana = period_ms (igual a dt_s*1000)
            esp_err_t e = rmt_step_pulse(cfg.rmt, dir, freq,
                                         (uint32_t)(cfg.dt_s * 1000.0f + 0.5f),
                                         cfg.pulse_us);
            if (e != ESP_OK) {
                ESP_LOGW(TAG, "rmt_step_pulse err=%d (v=%.1f sps)", (int)e, v);
            }
        } else {
            // Quieto: no emitir para evitar jitter de 1 paso
            vTaskDelay(cfg.period_ticks);
        }

        vTaskDelayUntil(&last, cfg.period_ticks);
    }
}

void motor_task_start(const motor_task_cfg_t *cfg_in)
{
    configASSERT(cfg_in);
    configASSERT(cfg_in->q_in != NULL);
    configASSERT(cfg_in->rmt  != NULL);
    configASSERT(cfg_in->period_ms > 0);

    motor_rt_t *cfg = pvPortMalloc(sizeof(*cfg));
    configASSERT(cfg);

    cfg->q_in        = cfg_in->q_in;
    cfg->rmt         = cfg_in->rmt;
    cfg->period_ticks= pdMS_TO_TICKS(cfg_in->period_ms);
    cfg->dt_s        = cfg_in->period_ms / 1000.0f;
    cfg->v_max       = (cfg_in->v_max_steps_s > 0) ? cfg_in->v_max_steps_s : 10000.0f;
    cfg->pulse_us    = (cfg_in->pulse_us > 0) ? cfg_in->pulse_us : 2;
    cfg->deadband    = (cfg_in->deadband_steps_s >= 0) ? cfg_in->deadband_steps_s : 1.0f;

    xTaskCreate(motor_task, "motor_task", 3*1024, cfg, tskIDLE_PRIORITY+2, NULL);
}
