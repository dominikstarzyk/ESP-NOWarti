#ifndef RGB_LED_H
#define RGB_LED_H

#include <stdint.h>
#include "esp_err.h"

// Default GPIO for RGB LED on ESP32-S3 DevKits
#define RGB_LED_GPIO 48

/**
 * @brief Initialize the RGB LED
 * 
 * @param gpio_num The GPIO pin connected to the RGB LED (WS2812)
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
 * @brief Set the LED to Red (Error state)
 */
void rgb_led_set_error(void);

/**
 * @brief Set the LED to Green (Drone Ready state)
 */
void rgb_led_set_ready(void);

/**
 * @brief Set the LED to Blue (Calibration state)
 */
void rgb_led_set_blue(void);

/**
 * @brief Set the LED to Orange (Warning/Not Calibrated state)
 */
void rgb_led_set_orange(void);

#endif // RGB_LED_H
