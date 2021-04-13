#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "i2c_module.h"

static const char *TAG = "I2C module";

static int is_init = 0;

void init_i2c_module()
{
    if (is_init)
    {
        return;
    }

    is_init = 1;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 18,
        .scl_io_num = 23,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 20000,
    };

    ESP_LOGD(TAG, "Install I2C driver");
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}