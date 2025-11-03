// control_task.c
#include "control_task.h"
#include "control_bus.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "control";




/*========================
 * Helpers
 *========================*/
static inline float fabsf_fast(float x) { return x < 0 ? -x : x; }

static inline float clampf(float x, float lim_abs) {
    if (x >  lim_abs) return  lim_abs;
    if (x < -lim_abs) return -lim_abs;
    return x;
}

static inline float slew_limit(float prev, float target, float slew_per_sample) {
    const float delta = target - prev;
    if (delta >  slew_per_sample) return prev + slew_per_sample;
    if (delta < -slew_per_sample) return prev - slew_per_sample;
    return target;
}

static inline bool in_enable_window(const control_params_t *cfg, float roll_deg, float roll_dps) {
    const float e_abs = fabsf_fast(cfg->ref_deg - roll_deg);
    if (e_abs > cfg->theta_enable_deg) return false;
    if (fabsf_fast(roll_dps) > cfg->dps_enable_max) return false; // evita armar con giro muy rápido
    return true;
}

/*========================
 * Estado global del controlador
 *========================*/
static control_state_t g_state = CTRL_DISABLED;
static control_params_t g_cfg = {
    .kp_int = 1.0f,
    .kd_int = 0.05f,
    .ref_deg = 90.0f,
    .theta_enable_deg  = 10.0f,
    .theta_disable_deg = 17.0f,
    .dps_enable_max    = 60.0f,
    .hold_enable_ms    = 200,
    .out_limit_abs     = 20000.0f,   // steps/s^2
    .out_slew_max      = 4000.0f     // steps/s^2 por sample
};

// Para FSM (ventana + hold-time)
static TickType_t g_window_enter_tick = 0;

/*========================
 * API pública (header)
 *========================*/
control_state_t control_get_state(void) { return g_state; }

void control_get_params(control_params_t *out) {
    if (out) *out = g_cfg;
}

void control_set_params(const control_params_t *p) {
    if (!p) return;
    control_params_t tmp = *p;

    // Sanitizado mínimo
    if (tmp.theta_enable_deg < 0.0f) tmp.theta_enable_deg = 0.0f;
    if (tmp.theta_disable_deg < tmp.theta_enable_deg + 1.0f)
        tmp.theta_disable_deg = tmp.theta_enable_deg + 1.0f;
    if (tmp.hold_enable_ms < 50)      tmp.hold_enable_ms = 50;
    if (tmp.out_limit_abs <= 0.0f)    tmp.out_limit_abs = 1.0f;
    if (tmp.out_slew_max  < 0.0f)     tmp.out_slew_max  = 0.0f;

    g_cfg = tmp; // snapshot completo

    ESP_LOGI(TAG,
        "Params set: Kp=%.3f Kd=%.3f ref=%.1f en/di=%.1f/%.1f hold=%ums lim=%.0f slew=%.0f/spl",
        g_cfg.kp_int, g_cfg.kd_int, g_cfg.ref_deg, g_cfg.theta_enable_deg, g_cfg.theta_disable_deg,
        (unsigned)g_cfg.hold_enable_ms, g_cfg.out_limit_abs, g_cfg.out_slew_max);
}

/*========================
 * Tarea principal
 *========================*/
static void control_task(void *arg) {
    ESP_LOGI(TAG, "Control task started");

    // Ritmo base del lazo de control
    const TickType_t period = pdMS_TO_TICKS(10); 
    TickType_t last = xTaskGetTickCount();

    // Estado interno del lazo PD
    float u_prev = 0.0f;   // salida previa (para slew)

    for (;;) {
        /* 1) Recibir COMANDOS (cambios de parámetros atómicos) */
        control_cmd_t cmd;
        if (q_ctrl_cmd && xQueueReceive(q_ctrl_cmd, &cmd, 0)) {
            if (cmd.type == CTRL_CMD_SET_PARAMS) {
                control_set_params(&cmd.params); // aplica snapshot entero
            }
        }

        /* 2) Leer IMU (timeout corto para no romper cadencia) */
        imu_msg_t m;
        bool have_sample = false;
        if (q_imu2ctrl && xQueueReceive(q_imu2ctrl, &m, pdMS_TO_TICKS(2))) {
            have_sample = true;
        }

        // Copia local coherente de configuración para este ciclo
        const control_params_t cfg = g_cfg;

        /* 3) FSM: DISABLED / ARMING / ENABLED (con histéresis y hold-time) */
        if (!have_sample) {
            if (g_state != CTRL_DISABLED) {
                ESP_LOGW(TAG, "No IMU sample -> DISABLED");
                g_state = CTRL_DISABLED;
            }
        } else {
            const float err_deg     = cfg.ref_deg - m.roll_deg;
            const float err_abs_deg = fabsf_fast(err_deg);
            const bool  inside      = in_enable_window(&cfg, m.roll_deg, m.roll_dps);

            switch (g_state) {
            default:
            case CTRL_DISABLED:
                if (inside) {
                    g_state = CTRL_ARMING;
                    g_window_enter_tick = xTaskGetTickCount();
                    ESP_LOGI(TAG, "DISABLED -> ARMING (|e|=%.1f°, w=%.1f°/s)", err_abs_deg, m.roll_dps);
                }
                break;

            case CTRL_ARMING: {
                if (!inside) {
                    g_state = CTRL_DISABLED;
                    ESP_LOGI(TAG, "ARMING -> DISABLED (salí de ventana)");
                } else {
                    const TickType_t now = xTaskGetTickCount();
                    const TickType_t hold_ticks = pdMS_TO_TICKS(cfg.hold_enable_ms);
                    if ((now - g_window_enter_tick) >= hold_ticks) {
                        g_state = CTRL_ENABLED;
                        ESP_LOGI(TAG, "ARMING -> ENABLED");
                        // Soft start: no reinyectar latigazo
                        u_prev = 0.0f;
                    }
                }
            } break;

            case CTRL_ENABLED:
                // Salgo si me voy del rango con histéresis
                if (err_abs_deg >= cfg.theta_disable_deg) {
                    g_state = CTRL_DISABLED;
                    ESP_LOGI(TAG, "ENABLED -> DISABLED (|e|=%.1f° >= %.1f°)", err_abs_deg, cfg.theta_disable_deg);
                    u_prev = 0.0f; // seguridad
                }
                break;

            case CTRL_FAULT:
                // No usamos aún condiciones de FAULT; tratamos como DISABLED con intento de recuperación
                if (inside) {
                    g_state = CTRL_ARMING;
                    g_window_enter_tick = xTaskGetTickCount();
                    ESP_LOGI(TAG, "FAULT -> ARMING (recupero)");
                }
                break;
            }
        }

        /* 4) Cálculo PD interno (SOLO si ENABLED) */
        const float E_DEAD_DEG = 0.5f;   // error angular chico
        const float W_DEAD_DPS = 1.0f;   // vel. angular chica
        const float U_DEAD     = 0.15f;  // mando chico -> apago
        const float U_BLEED    = 0.88f;  // “suelto” u_prev hacia 0 cuando estoy quieto
        static float w_filt = 0.0f;      // filtro 1er orden sobre roll_dps
        const float BETA = 0.85f;        // más cerca de 1 = más filtrado

        float u_pd = 0.0f;
        if (have_sample && g_state == CTRL_ENABLED) {
            // Derivada filtrada + zona muerta
            w_filt = BETA * w_filt + (1.0f - BETA) * m.roll_dps;
            float w_use = (fabsf(w_filt) < W_DEAD_DPS) ? 0.0f : w_filt;

            // Error con zona muerta
            float e = g_cfg.ref_deg - m.roll_deg;
            if (fabsf(e) < E_DEAD_DEG) e = 0.0f;

            // PD
            float u = g_cfg.kp_int * e + g_cfg.kd_int * (-w_use);

            // Deadband de mando
            if (fabsf(u) < U_DEAD) u = 0.0f;

            // Clamp + slew
            u = clampf(u, g_cfg.out_limit_abs);

            float dt = m.dt;
            float dt_fallback = (float)period / (float)configTICK_RATE_HZ;
            if (!(dt > 0.0f)) dt = dt_fallback;

            const float slew_per_sample = g_cfg.out_slew_max * dt;

            // Si estoy “quieto” (e=0 y w_use=0), drená u_prev a 0
            if (u == 0.0f) {
                u_prev *= U_BLEED;
                if (fabsf(u_prev) < U_DEAD) u_prev = 0.0f;
                u_pd = u_prev;
            } else {
                u_pd = slew_limit(u_prev, u, slew_per_sample);
                u_prev = u_pd;
            }
        } else {
            u_prev = 0.0f;
            u_pd   = 0.0f;
        }

        if (q_ctrl_out) {
            control_out_msg_t om = {
            .u_pd = u_pd,
            .dt   = (have_sample ? m.dt : ((float)period / (float)configTICK_RATE_HZ)),
            .tick = xTaskGetTickCount()
            };
            // cola tamaño 1 -> último dato siempre fresco
            (void)xQueueOverwrite(q_ctrl_out, &om);
        }

        /* 5) Debug cada ~200 ms */
        static uint32_t dbg_cnt = 0;
        if ((dbg_cnt++ % 40) == 0) {
            if (have_sample) {
                ESP_LOGI(TAG, "[st=%d] roll=%7.3f°  w=%7.3f°/s  u_pd=%8.1f",
                         (int)g_state, m.roll_deg, m.roll_dps, u_pd);
            } else {
                ESP_LOGI(TAG, "[st=%d] no-sample  u_pd=%8.1f", (int)g_state, u_pd);
            }
        }

        /* 6) Mantener cadencia estable */
        vTaskDelayUntil(&last, period);
    }
}

/*========================
 * Arranque de la tarea
 *========================*/
void control_task_start(UBaseType_t prio, uint32_t stack_words) {
    BaseType_t ok = xTaskCreate(control_task, "control_task",
                                stack_words, NULL, prio, NULL);
    ESP_ERROR_CHECK(ok == pdPASS ? ESP_OK : ESP_FAIL);
    ESP_LOGI(TAG, "control_task created (prio=%u, stack=%u words)",
             (unsigned)prio, (unsigned)stack_words);
}