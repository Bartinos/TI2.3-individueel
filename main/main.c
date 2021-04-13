#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include <string.h>
#include <stdio.h>
#include <time.h>

#include <sdkconfig.h>
#include <board.h>
#include <board_pins_config.h>
#include <esp_log.h>

#include "re_module.h"
#include "i2c_module.h"

#define I2C_MASTER_NUM I2C_NUM_0

static const char *TAG = "Main";

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing i2c");
    init_i2c_module();

    uint16_t *data_encoder_count = (uint16_t *)malloc(sizeof(uint16_t));
    uint16_t *data_encoder_difference = (uint16_t *)malloc(sizeof(uint16_t));
    bool *moved = (bool *)malloc(sizeof(bool));
    bool *pressed = (bool *)malloc(sizeof(bool));
    bool *clicked = (bool *)malloc(sizeof(bool));

    ESP_ERROR_CHECK(re_write_color(0, 0, 255)); // test RGB values

    while (1)
    {
        ESP_ERROR_CHECK(re_read_encoder_count(data_encoder_count));
        ESP_ERROR_CHECK(re_read_encoder_difference(data_encoder_difference));
        ESP_ERROR_CHECK(re_moved(moved));
        ESP_ERROR_CHECK(re_pressed(pressed));
        ESP_ERROR_CHECK(re_clicked(clicked));

        printf("Encoder Count: %d", (uint16_t)*data_encoder_count);
        printf(" Encoder Difference: %d", (uint16_t)*data_encoder_difference);
        printf(" Moved: %d", *moved);
        printf(" Pressed: %d", *pressed);
        printf(" Clicked: %d\n", *clicked);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
