#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Inicializa Bluetooth Classic SPP (usa callbacks del stack)
esp_err_t bt_spp_start(const char *dev_name);

// Enviar texto por SPP (telemetría/echo)
void bt_spp_write(const char *s);

#ifdef __cplusplus
}
#endif
