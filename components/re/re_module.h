#ifndef RE_MODULE_H
#define RE_MODULE_H

#include "esp_log.h"

esp_err_t re_read_encoder_count(uint16_t *data);
esp_err_t re_read_encoder_difference(uint16_t *data);
esp_err_t re_clicked(bool *clicked);
esp_err_t re_pressed(bool *pressed);
esp_err_t re_moved(bool *moved);
esp_err_t re_write_color(uint8_t red, uint8_t green, uint8_t blue);

#endif