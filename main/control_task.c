#include "control_task.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "CTRL";

// Guardamos una copia local de la config
typedef struct {
    QueueHandle_t q_out;
    TickType_t    period_ticks;
    float         A;
} ctrl_cfg_runtime_t;

static void control_task(void *arg)
{
    ctrl_cfg_runtime_t cfg = *(ctrl_cfg_runtime_t*)arg;
    vPortFree(arg); // liberar copia heap
    motor_cmd_t cmd;

    // Perfil: +A 1s -> 0 1s -> -A 1s -> 0 1s -> (repite)
    // Estados: 0:+A, 1:0, 2:-A, 3:0
    const uint32_t slot_ms   = 1000;
    const uint32_t step_ms   = (uint32_t)(cfg.period_ticks * portTICK_PERIOD_MS);
    const uint32_t slot_steps = (slot_ms + step_ms/2) / step_ms; // iteraciones por slot

    uint32_t k = 0;    // contador dentro del slot actual
    int state = 0;     // 0..3
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        // Selección de aceleración según estado
        switch (state) {
            case 0: cmd.accel_steps_s2 = +cfg.A; break;
            case 1: cmd.accel_steps_s2 = 0.0f;   break;
            case 2: cmd.accel_steps_s2 = -cfg.A; break;
            case 3: cmd.accel_steps_s2 = 0.0f;   break;
            default: cmd.accel_steps_s2 = 0.0f;  break;
        }

        // Enviar SIEMPRE el último valor (overwrite evita colas largas)
        (void) xQueueOverwrite(cfg.q_out, &cmd);

        // Avance de perfil
        k++;
        if (k >= slot_steps) {
            k = 0;
            state = (state + 1) & 3; // 0->1->2->3->0
            ESP_LOGI(TAG, "Perfil CTRL state=%d, a=%.0f steps/s^2", state, cmd.accel_steps_s2);
        }

        vTaskDelayUntil(&last, cfg.period_ticks);
    }
}

void control_task_start(const control_task_cfg_t *cfg_in)
{
    configASSERT(cfg_in);
    configASSERT(cfg_in->q_out != NULL);
    configASSERT(cfg_in->period_ms > 0);

    // Copia runtime en heap para pasar al task
    ctrl_cfg_runtime_t *cfg = pvPortMalloc(sizeof(*cfg));
    configASSERT(cfg);
    cfg->q_out = cfg_in->q_out;
    cfg->period_ticks = pdMS_TO_TICKS(cfg_in->period_ms);
    cfg->A = (cfg_in->A_steps_s2 != 0.0f) ? cfg_in->A_steps_s2 : 3000.0f;

    xTaskCreate(control_task, "control_task", 3*1024, cfg, tskIDLE_PRIORITY+2, NULL);
}
