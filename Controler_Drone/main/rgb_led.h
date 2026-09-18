#ifndef RGB_LED_H
#define RGB_LED_H

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Initialize the built-in RGB LED on the specified GPIO
 * 
 * @param gpio_num GPIO number connected to the WS2812 LED
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rgb_led_init(int gpio_num);

/**
 * @brief Set the color of the RGB LED
 * 
 * @param red Red component (0-255)
 * @param green Green component (0-255)
 * @param blue Blue component (0-255)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rgb_led_set_color(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Set LED to RED (Error)
 */
void rgb_led_set_error(void);

/**
 * @brief Set LED to GREEN (Ready/Normal)
 */
void rgb_led_set_ready(void);

/**
 * @brief Set LED to BLUE (Calibration)
 */
void rgb_led_set_blue(void);

/**
 * @brief Set LED to ORANGE
 */
void rgb_led_set_orange(void);

#endif // RGB_LED_H
