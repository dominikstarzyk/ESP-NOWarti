#ifndef ESP_NOW_RECEIVE_H
#define ESP_NOW_RECEIVE_H

#include <stdint.h>
#include <stdbool.h>

#define ENABLE_SNIFF
//#define DEBUG_LOGS
#define ENCRYPTION_MODE

typedef struct __attribute__((packed)) {
    float roll;
    float pitch;
    float yaw;
    float throttle;
    uint16_t crc;
} espnow_data_t;

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief Initialize WiFi and ESP-NOW for receiving data
 * @param motor_queue Queue to send motor speed updates to
 */
void esp_now_receive_init(QueueHandle_t motor_queue);

#endif // ESP_NOW_RECEIVE_H
