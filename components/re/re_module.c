#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "driver/i2c.h"

#define RE_STATUS 0x01                 // address of status of the rotary encoder, 0 bit moved, 1 bit pressed, 2 bit clicked
#define RE_ENCODER_COUNT_MSB 0x05      // address of MSB number of ticks twisted
#define RE_ENCODER_COUNT_LSB 0x06      // address of LSB number of ticks twisted
#define RE_ENCODER_DIFFERENCE_MSB 0x07 // address of MSB number of ticks twisted since last movement
#define RE_ENCODER_DIFFERENCE_LSB 0x08 // address of LSB number of ticks twisted since last movement
#define RE_LED_BRIGHTNESS_RED 0x0D     // address representing the brightness of RED LED
#define RE_LED_BRIGHTNESS_GREEN 0x0E   // address representing the brightness of GREEN LED
#define RE_LED_BRIGHTNESS_BLUE 0x0F    // address representing the brightness of BLUE LED
#define RE_ADDRESS 0x3F                // address of rotary encoder

#define ACK_CHECK_EN 0x1  // I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0 // I2C master will not check ack from slave
#define ACK_VAL 0x0       // I2C ack value
#define NACK_VAL 0x1      // I2C nack value

#define DATA_CLEAR 0x00   // indicates data meant to be cleared

#define STATUS_ENCODER_MOVED 0   // bit position of moved bit in status byte
#define STATUS_ENCODER_PRESSED 1 // bit position of pressed bit in status byte
#define STATUS_ENCODER_CLICKED 2 // bit position of clicked bit in status byte

static const char *TAG = "RE module";

/**
 * @brief reads two desired bytes from the RE module and stores it in one of its parameters
 * 
 * @param data contains the retrieved information
 *  @param reg_msb the address of the most significant byte
 *  @param reg_lsb the address of the least significant byte
 * 
 *  @return error type, should be ESP_OK when successful.
 */
static esp_err_t _re_read_register16(uint16_t *data, uint8_t reg_msb, uint8_t reg_lsb)
{
    uint8_t re_msb = 0;
    uint8_t re_lsb = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // build message
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RE_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_lsb, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RE_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &re_lsb, ACK_VAL);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RE_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_msb, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RE_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &re_msb, NACK_VAL);

    i2c_master_stop(cmd);                                                          // end message
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS); // send message
    i2c_cmd_link_delete(cmd);

    *data = (re_msb << 8) | re_lsb;

    return ret;
}

/**
 * @brief reads the desired byte from the RE module and stores it in one of its parameters
 * 
 * @param data contains the retrieved information
 * @param reg the address of the desired byte
 * 
 * @return error type, should be ESP_OK when successful.
 */
static esp_err_t _re_read_register8(uint8_t *data, uint8_t reg)
{
    uint8_t value = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // build message
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RE_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RE_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &value, NACK_VAL);

    i2c_master_stop(cmd);                                                          // end message
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS); // send message
    i2c_cmd_link_delete(cmd);

    *data = value;

    return ret;
}

/**
 * @brief writes a byte to the desired address of the RE module
 * 
 * @param data a byte which contains the write data
 * @param reg the address to write to
 *  
 * @return error type, should be ESP_OK when successful.
 */
static esp_err_t _re_write_register8(uint8_t data, uint8_t reg)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // build message
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RE_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);

    i2c_master_stop(cmd);                                                          // end message
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS); // send message
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief writes two bytes to the desired addresses of the RE module
 * 
 * @param data contains the write data
 *  @param reg_msb the address of the most significant byte
 *  @param reg_lsb the address of the least significant byte
 * 
 *  @return error type, should be ESP_OK when successful.
 */
static esp_err_t _re_write_register16(uint16_t data, uint8_t reg_msb, uint8_t reg_lsb)
{

    uint8_t re_msb = data >> 8;     // store the first 8 bits
    uint8_t re_lsb = data & 0x00FF; // store the last 8 bits
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // build message
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RE_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_msb, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, re_msb, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RE_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_lsb, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, re_lsb, ACK_CHECK_EN);

    i2c_master_stop(cmd);                                                          // end message
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS); // send message
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief retrieves the encoder count stored in the rotary encoder
 * 
 * @param data will hold the retrieved data
 * 
 *  @return error type, should be ESP_OK when successful.
 */
esp_err_t re_read_encoder_count(uint16_t *data)
{
    return _re_read_register16(data, RE_ENCODER_COUNT_LSB, RE_ENCODER_COUNT_MSB);
}

/**
 * @brief retrieves the encoder difference since last clear,
 * gets cleared every reading
 * 
 * @param data will hold the retrieved data
 * 
 *  @return error type, should be ESP_OK when successful.
 */
esp_err_t re_read_encoder_difference(uint16_t *data)
{
    char err = _re_read_register16(data, RE_ENCODER_DIFFERENCE_LSB, RE_ENCODER_DIFFERENCE_MSB);
    _re_write_register16(DATA_CLEAR, RE_ENCODER_DIFFERENCE_LSB, RE_ENCODER_DIFFERENCE_MSB);

    return err;
}

/**
 * @brief retrieves the encoder status stored in the rotary encoder,
 * reads the moved bit and clears it
 * 
 * @param moved will hold the retrieved data
 * 
 *  @return error type, should be ESP_OK when successful.
 */
esp_err_t re_moved(bool *moved)
{
    uint8_t *data = (uint8_t *)malloc(sizeof(uint8_t));
    char err = _re_read_register8(&data, RE_STATUS);

    *moved = (int)data & (int)(1 << STATUS_ENCODER_MOVED);

    uint8_t clear = (int)data & (int)0b11111110; // clear the last bit indicating moved value
    _re_write_register8(clear, RE_STATUS);

    return err;
}

/**
 * @brief retrieves the encoder status stored in the rotary encoder,
 * reads the pressed bit and clears it
 * 
 * @param pressed will hold the retrieved data
 * 
 *  @return error type, should be ESP_OK when successful.
 */
esp_err_t re_pressed(bool *pressed)
{
    uint8_t *data = (uint8_t *)malloc(sizeof(uint8_t));
    char err = _re_read_register8(&data, RE_STATUS);

    *pressed = (int)data & (int)(1 << STATUS_ENCODER_PRESSED);

    uint8_t clear = (int)data & (int)0b11111101; // clear the second last bit indicating pressed value
    _re_write_register8(clear, RE_STATUS);

    return err;
}

/**
 * @brief retrieves the encoder status stored in the rotary encoder,
 * reads the clicked bit and clears it
 * 
 * @param clicked will hold the retrieved data
 * 
 *  @return error type, should be ESP_OK when successful.
 */
esp_err_t re_clicked(bool *clicked)
{
    uint8_t *data = (uint8_t *)malloc(sizeof(uint8_t));
    char err = _re_read_register8(&data, RE_STATUS);

    *clicked = (int)data & (int)(1 << STATUS_ENCODER_CLICKED);

    uint8_t clear = (int)data & (int)0b11111011; // clear the third last bit indicating click value
    _re_write_register8(clear, RE_STATUS);

    return err;
}

/**
 * @brief write the RGB values to the rotary encoder changing the color output
 * 
 * @param red contains the red value to write
 * @param green contains the green value to write
 * @param blue contains the blue value to write
 * 
 *  @return error type, should be ESP_OK when successful.
 */
esp_err_t re_write_color(uint8_t red, uint8_t green, uint8_t blue)
{
    char err = 0;
    if ((err = _re_write_register8(red, RE_LED_BRIGHTNESS_RED)) != 0)
        return err;
    if ((err = _re_write_register8(green, RE_LED_BRIGHTNESS_GREEN)) != 0)
        return err;
    if ((err = _re_write_register8(blue, RE_LED_BRIGHTNESS_BLUE)) != 0)
        return err;

    return 0;
}
