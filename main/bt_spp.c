#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>
#include <math.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "control_task.h"
#include "control_bus.h"

static const char *TAG = "bt_spp";

// ===== Config =====
#define LINE_MAX        128      // tamaño de comando entrante
#define REPLY_MAX       192
#define USE_SPP_ENHANCED 0       // usar SPP legacy (estable)

// ===== Estado SPP =====
static uint32_t s_spp_handle = 0;
static char     s_linebuf[LINE_MAX];
static size_t   s_linepos = 0;

// Cola de comandos (líneas) y tarea que los procesa
static QueueHandle_t s_cmdq = NULL;
static TaskHandle_t  s_cmdtask = NULL;

// Cola del control (extern)
extern QueueHandle_t q_ctrl_cmd;

// =================== helpers ===================
static void trim(char *s){
    char *p = s;
    while (*p && isspace((unsigned char)*p)) p++;
    if (p != s) memmove(s, p, strlen(p)+1);
    size_t n = strlen(s);
    while (n>0 && isspace((unsigned char)s[n-1])) s[--n] = 0;
}

// respuesta SPP (desde la tarea, no desde callbacks)
static void replyf(const char *fmt, ...){
    if (!s_spp_handle) return;
    char buf[REPLY_MAX];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    // asegurar CRLF al final si no lo incluyeron
    if (n+2 < (int)sizeof(buf)) {
        if (n < 2 || !(buf[n-2] == '\r' && buf[n-1] == '\n')) {
            buf[n++] = '\r';
            buf[n++] = '\n';
            buf[n] = 0;
        }
    }
    esp_spp_write(s_spp_handle, n, (uint8_t*)buf);
}

void bt_spp_write(const char *s){
    if (s && s_spp_handle) {
        size_t n = strlen(s);
        esp_spp_write(s_spp_handle, n, (uint8_t*)s);
    }
}

// =================== Ejecutar comando ===================
static float clampf(float x, float lo, float hi){
    if (isnan(x) || isinf(x)) return 0.0f;
    if (x < lo) x = lo;
    if (x > hi) x = hi;
    return x;
}

static void handle_line(char *line){
    trim(line);
    if (!*line) return;

    if (!strncasecmp(line, "get", 3)){
        control_params_t p; control_get_params(&p);
        replyf("OK params: kp=%.3f kd=%.3f ref=%.1f en=%.1f di=%.1f hold=%u lim=%.0f slew=%.0f",
               p.kp_int, p.kd_int, p.ref_deg, p.theta_enable_deg, p.theta_disable_deg,
               (unsigned)p.hold_enable_ms, p.out_limit_abs, p.out_slew_max);
        return;
    }

    if (!strncasecmp(line, "state", 5)){
        control_state_t st = control_get_state();
        replyf("OK state=%d", (int)st);
        return;
    }

    if (!strncasecmp(line, "help", 4)){
        replyf("cmds: get | state | set kp|kd|ref|enwin|diwin|hold|lim|slew <val>");
        return;
    }

    if (!strncasecmp(line, "set", 3)){
        char key[16]; float val = 0;
        if (sscanf(line+3, " %15s %f", key, &val) >= 2){
            // clamps razonables para evitar valores patológicos
            if      (!strcasecmp(key,"kp"))    val = clampf(val, 0.0f, 10.0f);
            else if (!strcasecmp(key,"kd"))    val = clampf(val, 0.0f, 10.0f);
            else if (!strcasecmp(key,"ref"))   val = clampf(val, -180.0f, 180.0f);
            else if (!strcasecmp(key,"enwin")) val = clampf(val, 0.0f, 60.0f);
            else if (!strcasecmp(key,"diwin")) val = clampf(val, 0.0f, 90.0f);
            else if (!strcasecmp(key,"hold"))  val = clampf(val, 0.0f, 5000.0f);
            else if (!strcasecmp(key,"lim"))   val = clampf(val, 0.0f, 100000.0f);
            else if (!strcasecmp(key,"slew"))  val = clampf(val, 0.0f, 200000.0f);

            control_params_t p; control_get_params(&p);
            if      (!strcasecmp(key,"kp"))    p.kp_int = val;
            else if (!strcasecmp(key,"kd"))    p.kd_int = val;
            else if (!strcasecmp(key,"ref"))   p.ref_deg = val;
            else if (!strcasecmp(key,"enwin")) p.theta_enable_deg = val;
            else if (!strcasecmp(key,"diwin")) p.theta_disable_deg = val;
            else if (!strcasecmp(key,"hold"))  p.hold_enable_ms   = (uint32_t)(val + 0.5f);
            else if (!strcasecmp(key,"lim"))   p.out_limit_abs    = val;
            else if (!strcasecmp(key,"slew"))  p.out_slew_max     = val;
            else { replyf("ERR unknown key"); return; }

            control_cmd_t c = { .type = CTRL_CMD_SET_PARAMS, .params = p };
            if (q_ctrl_cmd && xQueueOverwrite(q_ctrl_cmd, &c) == pdPASS){
                replyf("OK set %s=%.3f", key, val);
            } else {
                replyf("ERR queue");
            }
            return;
        }
        replyf("ERR syntax");
        return;
    }

    replyf("ERR unknown. try: help");
}

// =================== Tarea de comandos ===================
static void spp_cmd_task(void *arg){
    char line[LINE_MAX];
    for(;;){
        if (xQueueReceive(s_cmdq, &line, portMAX_DELAY) == pdPASS){
            handle_line(line);
        }
    }
}

// =================== GAP ===================
static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param){
    switch(event){
    case ESP_BT_GAP_PIN_REQ_EVT: {
        esp_bt_pin_code_t pin = { '1','2','3','4' };
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
    } break;
    default:
        break;
    }
}

// =================== SPP callbacks ===================
static void spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "SPP init");
        esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "SPP_SERVER");
        break;

    case ESP_SPP_START_EVT:
        ESP_LOGI(TAG, "SPP server started");
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        s_spp_handle = param->srv_open.handle;
        s_linepos = 0;
        ESP_LOGI(TAG, "Connectado");
        replyf("Escribir 'help' para ver comandos");
        break;

    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "Desconectado");
        s_spp_handle = 0;
        s_linepos = 0;
        break;

    case ESP_SPP_DATA_IND_EVT: {
        const uint8_t *d = param->data_ind.data;
        uint16_t n = param->data_ind.len;
        for (uint16_t i = 0; i < n; i++){
            char c = (char)d[i];
            if (c == '\r') continue;        // ignorar CR
            if (c == '\n'){                  // fin de línea -> a la cola
                s_linebuf[s_linepos] = 0;
                if (s_cmdq) {
                    // enviamos una copia para la tarea
                    char line[LINE_MAX];
                    strncpy(line, s_linebuf, LINE_MAX);
                    line[LINE_MAX-1] = 0;
                    xQueueSend(s_cmdq, &line, 0);
                }
                s_linepos = 0;
            } else if (s_linepos + 1 < sizeof(s_linebuf)){
                s_linebuf[s_linepos++] = c;
            }
        }
    } break;

    default:
        break;
    }
}

// =================== API pública ===================
esp_err_t bt_spp_start(const char *dev_name)
{
    // NVS
    esp_err_t nvs_check = nvs_flash_init();
    if (nvs_check == ESP_ERR_NVS_NO_FREE_PAGES || nvs_check == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Cola + tarea de comandos (si no existen)
    if (!s_cmdq) {
        s_cmdq = xQueueCreate(4, sizeof(char[LINE_MAX]));
        ESP_RETURN_ON_FALSE(s_cmdq != NULL, ESP_ERR_NO_MEM, TAG, "cmdq");
    }
    if (!s_cmdtask) {
        BaseType_t ok = xTaskCreate(spp_cmd_task, "spp_cmd_task", 3*1024, NULL, tskIDLE_PRIORITY+2, &s_cmdtask);
        ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "cmdtask");
    }

    // Controller
    esp_bt_controller_status_t cst = esp_bt_controller_get_status();
    if (cst == ESP_BT_CONTROLLER_STATUS_IDLE) {
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ESP_RETURN_ON_ERROR(esp_bt_controller_init(&bt_cfg), TAG, "ctlr init");
        ESP_RETURN_ON_ERROR(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT), TAG, "ctlr en");
    } else if (cst == ESP_BT_CONTROLLER_STATUS_INITED) {
        ESP_RETURN_ON_ERROR(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT), TAG, "ctlr en2");
    } // ENABLED: seguir

    // Bluedroid
    esp_bluedroid_status_t bst = esp_bluedroid_get_status();
    if (bst == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        ESP_RETURN_ON_ERROR(esp_bluedroid_init(),  TAG, "bd init");
        ESP_RETURN_ON_ERROR(esp_bluedroid_enable(), TAG, "bd en");
    } else if (bst == ESP_BLUEDROID_STATUS_INITIALIZED) {
        ESP_RETURN_ON_ERROR(esp_bluedroid_enable(), TAG, "bd en2");
    }

    // GAP + nombre
    ESP_RETURN_ON_ERROR(esp_bt_gap_register_callback(gap_cb), TAG, "gap cb");
    if (dev_name && *dev_name) {
        ESP_RETURN_ON_ERROR(esp_bt_dev_set_device_name(dev_name), TAG, "set name");
    }
    ESP_RETURN_ON_ERROR(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE),
                        TAG, "scan mode");

    // SPP (legacy)
    ESP_ERROR_CHECK(esp_spp_register_callback(spp_cb));
#if USE_SPP_ENHANCED
    ESP_RETURN_ON_ERROR(esp_spp_enhanced_init(ESP_SPP_MODE_CB), TAG, "spp init");
#else
    ESP_RETURN_ON_ERROR(esp_spp_init(ESP_SPP_MODE_CB), TAG, "spp init");
#endif

    ESP_LOGI(TAG, "BT SPP ready");
    return ESP_OK;
}

esp_err_t bt_spp_stop(void)
{
    // apagar Bluedroid / controller
    esp_bluedroid_status_t bst = esp_bluedroid_get_status();
    if (bst == ESP_BLUEDROID_STATUS_ENABLED) {
        ESP_ERROR_CHECK(esp_bluedroid_disable());
        bst = esp_bluedroid_get_status();
    }
    if (bst == ESP_BLUEDROID_STATUS_INITIALIZED) {
        ESP_ERROR_CHECK(esp_bluedroid_deinit());
    }

    esp_bt_controller_status_t cst = esp_bt_controller_get_status();
    if (cst == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        ESP_ERROR_CHECK(esp_bt_controller_disable());
        cst = esp_bt_controller_get_status();
    }
    if (cst == ESP_BT_CONTROLLER_STATUS_INITED) {
        ESP_ERROR_CHECK(esp_bt_controller_deinit());
    }
    return ESP_OK;
}
