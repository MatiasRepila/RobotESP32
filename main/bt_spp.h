#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bt_spp_start(const char *dev_name);

#ifdef __cplusplus
}
#endif