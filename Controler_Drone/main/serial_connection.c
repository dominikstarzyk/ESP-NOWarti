#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "serial_connection.h"
#include "esp_now_transmite.h"
#include "joystick.h"

static const char *TAG = "serial_conn";

#if USE_SERIAL_CONNECTION
static void serial_task(void *pvParameters)
{
    char line[128];
    int line_idx = 0;
    espnow_data_t drone_data;

    // Set stdin to non-buffered to get characters immediately
    setvbuf(stdin, NULL, _IONBF, 0);

    while (1) {
        // Read one character from stdin
        int c = getchar();

        // If no character is available, yield
        if (c == EOF || c == 0xFF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Check for newline (Enter key)
        if (c == '\n' || c == '\r') {
            if (line_idx > 0) {
                line[line_idx] = '\0';
                ESP_LOGI(TAG, "Input Received: [%s]", line);

                // Parse CSV format: roll,pitch,yaw,throttle
                if (sscanf(line, "%f,%f,%f,%f", 
                           &drone_data.roll, &drone_data.pitch, 
                           &drone_data.yaw, &drone_data.throttle) == 4) {
                    
                    ESP_LOGI(TAG, "Command Parsed -> R:%.2f P:%.2f Y:%.2f T:%.2f", 
                             drone_data.roll, drone_data.pitch, drone_data.yaw, drone_data.throttle);
                    
                    if (espnow_queue != NULL) {
                        xQueueSend(espnow_queue, &drone_data, portMAX_DELAY);
                    }
                } else {
                    ESP_LOGW(TAG, "Invalid format! Usage: roll,pitch,yaw,throttle (e.g. 10.0,0,0,50)");
                }
                line_idx = 0; // Reset for next line
            }
        } else {
            // Store character if message isn't too long
            if (line_idx < sizeof(line) - 1) {
                line[line_idx++] = (char)c;
            }
        }
    }
}
#endif

void serial_connection_init(void)
{
#if USE_SERIAL_CONNECTION
    ESP_LOGI(TAG, "USB Console (stdin) Input Task started");
    xTaskCreate(serial_task, "serial_task", 4096, NULL, 5, NULL);
#else
    ESP_LOGI(TAG, "Serial connection disabled (Joystick mode active in main)");
#endif
}
