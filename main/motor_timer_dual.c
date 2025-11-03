#include "motor_timer_dual.h"
#include "control_bus.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include <stdatomic.h>
#include <math.h>
#include "control_task.h"


// ========= Estado/control común =========
static const char *TAG = "motor_timer_dual";

extern QueueHandle_t q_ctrl_out;   // última salida del control (u_pd, dt, tick)

static uint32_t s_isr_hz      = 0;
static uint32_t s_pulse_ticks = 2;

// Pines
static gpio_num_t s_stepL = GPIO_NUM_NC, s_dirL = GPIO_NUM_NC;
static gpio_num_t s_stepR = GPIO_NUM_NC, s_dirR = GPIO_NUM_NC;

// DDS fijo Q24
#define DDS_Q   24
#define DDS_ONE (1u << DDS_Q)

// Consignas tarea->ISR (enteras, atómicas)
static _Atomic uint32_t s_inc_L = 0;   // incremento Q24 por tick
static _Atomic uint32_t s_inc_R = 0;
static _Atomic int      s_dir_L = 0;   // 0/1
static _Atomic int      s_dir_R = 0;

// Acumuladores DDS + duración de pulso
static uint32_t s_phase_L = 0, s_phase_R = 0;
static int      s_pulse_rem_L = 0, s_pulse_rem_R = 0;

// ===== Helpers =====
static inline uint32_t freq_to_inc(float freq_steps_s){
    if (freq_steps_s <= 0.0f || s_isr_hz == 0) return 0;
    // inc = round(freq / isr_hz * 2^Q)
    double inc = (double)freq_steps_s * (double)DDS_ONE / (double)s_isr_hz;
    if (inc < 0.0) inc = 0.0;
    if (inc > 4294967295.0) inc = 4294967295.0;
    return (uint32_t)(inc + 0.5);
}

void motor_timer_set_L(float freq_steps_s, int dir_level){
    atomic_store_explicit(&s_inc_L, freq_to_inc(freq_steps_s), memory_order_relaxed);
    atomic_store_explicit(&s_dir_L, dir_level ? 1 : 0,         memory_order_relaxed);
}
void motor_timer_set_R(float freq_steps_s, int dir_level){
    atomic_store_explicit(&s_inc_R, freq_to_inc(freq_steps_s), memory_order_relaxed);
    atomic_store_explicit(&s_dir_R, dir_level ? 1 : 0,         memory_order_relaxed);
}

// ISR GPTimer: **sin floats** (sólo entero)
static bool IRAM_ATTR on_timer(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*)
{
    // Mantener pulso alto si corresponde
    if (s_pulse_rem_L > 0 && --s_pulse_rem_L == 0) gpio_set_level(s_stepL, 0);
    if (s_pulse_rem_R > 0 && --s_pulse_rem_R == 0) gpio_set_level(s_stepR, 0);

    // Snapshot consignas (enteras, atómicas)
    const uint32_t incL = atomic_load_explicit(&s_inc_L, memory_order_relaxed);
    const uint32_t incR = atomic_load_explicit(&s_inc_R, memory_order_relaxed);
    const int      dL   = atomic_load_explicit(&s_dir_L, memory_order_relaxed);
    const int      dR   = atomic_load_explicit(&s_dir_R, memory_order_relaxed);

    // DIR (setup time sobra)
    if (s_dirL != GPIO_NUM_NC) gpio_set_level(s_dirL, dL);
    if (s_dirR != GPIO_NUM_NC) gpio_set_level(s_dirR, dR);

    // DDS entero: a lo sumo un pulso por tick y por motor
    if (incL) {
        s_phase_L += incL;
        if (s_phase_L >= DDS_ONE) {
            s_phase_L -= DDS_ONE;
            gpio_set_level(s_stepL, 1);
            s_pulse_rem_L = (int)s_pulse_ticks;
            if (s_pulse_rem_L <= 0) s_pulse_rem_L = 1;
        }
    }
    if (incR) {
        s_phase_R += incR;
        if (s_phase_R >= DDS_ONE) {
            s_phase_R -= DDS_ONE;
            gpio_set_level(s_stepR, 1);
            s_pulse_rem_R = (int)s_pulse_ticks;
            if (s_pulse_rem_R <= 0) s_pulse_rem_R = 1;
        }
    }
    return true; // rearmar
}

void motor_timer_dual_start(gpio_num_t step_L, gpio_num_t dir_L,
                            gpio_num_t step_R, gpio_num_t dir_R,
                            uint32_t isr_hz, uint32_t pulse_ticks)
{
    s_stepL = step_L; s_dirL = dir_L;
    s_stepR = step_R; s_dirR = dir_R;
    s_isr_hz = isr_hz;
    s_pulse_ticks = pulse_ticks ? pulse_ticks : 1;

    // GPIOs salida
    gpio_config_t io = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<step_L) | (1ULL<<step_R) |
                        (dir_L!=GPIO_NUM_NC ? (1ULL<<dir_L):0) |
                        (dir_R!=GPIO_NUM_NC ? (1ULL<<dir_R):0),
        .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(step_L, 0); gpio_set_level(step_R, 0);
    if (dir_L!=GPIO_NUM_NC) gpio_set_level(dir_L, 0);
    if (dir_R!=GPIO_NUM_NC) gpio_set_level(dir_R, 0);

    // GPTimer
    gptimer_handle_t tmr = NULL;
    gptimer_config_t tcfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = s_isr_hz,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&tcfg, &tmr));

    gptimer_event_callbacks_t cbs = { .on_alarm = on_timer };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(tmr, &cbs, NULL));

    gptimer_alarm_config_t acfg = {
        .reload_count = 0,
        .alarm_count = 1,                  // 1 tick → ISR a s_isr_hz
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(tmr, &acfg));
    // Importante: no loguear entre enable y start para evitar ISR en medio del log
    ESP_ERROR_CHECK(gptimer_enable(tmr));
    ESP_ERROR_CHECK(gptimer_start(tmr));

    ESP_LOGI(TAG, "GPTimer dual STEP: isr_hz=%u, pulse_ticks=%u",
             (unsigned)s_isr_hz, (unsigned)s_pulse_ticks);
}

// ===== Task ligera: integra u_pd -> vel y alimenta ambos motores =====
static float s_k_accel = 1000.0f;   // [steps/s^2] por 1.0 de u_pd
static float s_vmax    = 8000.0f;   // [steps/s]
static float s_amax    = 20000.0f;  // [steps/s^2]

void motor_set_k_accel(float k){ s_k_accel = k; }
void motor_set_limits(float vmax, float amax){ s_vmax=vmax; s_amax=amax; }

static inline float clamp_abs(float x, float lim){
    return (x > lim) ? lim : (x < -lim ? -lim : x);
}

static void motor_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz
    TickType_t last = xTaskGetTickCount();

    control_out_msg_t m = {0};
    float vel = 0.0f;

    // deadband para no “titiritear” a freq bajísimas
    const float F_MIN_HZ = 5.0f;

    for(;;){
        // Tomo el último u_pd disponible
        while (q_ctrl_out && xQueueReceive(q_ctrl_out, &m, 0)==pdPASS) { /* keep last */ }

        // Si no estoy habilitado, FRENO TODO y sigo al próximo ciclo
        control_state_t st = control_get_state();   // CTRL_ENABLED = 2 en tu enum
        if (st != CTRL_ENABLED) {
            vel = 0.0f;
            motor_timer_set_L(0.0f, 1);
            motor_timer_set_R(0.0f, 1);
            vTaskDelayUntil(&last, period);
            continue;
        }

        // Integración u_pd -> velocidad (con límites)
        float dt = (m.dt > 0 ? m.dt : 0.01f);
        float a  = s_k_accel * m.u_pd;
        if (a >  s_amax) a =  s_amax;
        if (a < -s_amax) a = -s_amax;

        vel += a * dt;
        if (vel >  s_vmax) vel =  s_vmax;
        if (vel < -s_vmax) vel = -s_vmax;

        // Comando a generador
        float f = fabsf(vel);
        if (f < F_MIN_HZ) f = 0.0f;   // deadband

        int dir = (vel >= 0.0f) ? 1 : 0;
        motor_timer_set_L(f, dir);
        motor_timer_set_R(f, dir);

        vTaskDelayUntil(&last, period);
    }
}

void motor_task_timer_start(UBaseType_t prio, uint32_t stack_words)
{
    // Podés fijarla al core 1 si querés (último argumento = 1)
    BaseType_t ok = xTaskCreatePinnedToCore(motor_task, "motor_task_timer",
                                            stack_words, NULL, prio, NULL, 1);
    ESP_ERROR_CHECK(ok == pdPASS ? ESP_OK : ESP_FAIL);
}
