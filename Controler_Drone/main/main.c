/* ESP-NOW Basic Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now_transmite.h"
#include "esp_crc.h"
#include "serial_connection.h"
#include "joystick.h"
#include "rgb_led.h"

#define RGB_LED_GPIO 48

#define ESPNOW_MAXDELAY 512

static const char *TAG = "esp_now_basic";

/* WiFi init */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi
    wifi_init();

    // Initialize ESPNOW
    app_espnow_init();

    // Initialize RGB LED
    rgb_led_init(RGB_LED_GPIO);
    rgb_led_set_ready(); // Default to Green

#if USE_SERIAL_CONNECTION
    // Initialize Serial Connection
    serial_connection_init();
#else
    // Initialize Joystick
    joystick_init();
    xTaskCreate(joystick_task, "joystick_task", 4096, NULL, 5, NULL);
#endif

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
