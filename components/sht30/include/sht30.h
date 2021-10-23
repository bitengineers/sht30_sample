#pragma once

#include "esp_err.h"

esp_err_t sht30_init(void);
esp_err_t sht30_deinit(void);

esp_err_t sht30_read_status(uint16_t *out);
esp_err_t sht30_reset_status(void);
esp_err_t sht30_softreset(void);
esp_err_t sht30_read_measured_values(uint16_t *temperature, uint16_t *humidity);
esp_err_t sht30_stop_measurement(void);
esp_err_t sht30_heater(bool b);

float sht30_calc_celsius(uint16_t temp);
float sht30_calc_relative_humidity(uint16_t hum);

