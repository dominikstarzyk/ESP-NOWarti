#ifndef ESP_NOW_TRANSMITE_H
#define ESP_NOW_TRANSMITE_H

#define DEBUG_LOGS
#define ENCRYPTION_MODE

#include <stdint.h>
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/* ESPNOW data structure */
typedef struct __attribute__((packed)) {
    float roll;
    float pitch;
    float yaw;
    float throttle;
    uint16_t crc;
} espnow_data_t;

extern uint8_t mac_receiver[ESP_NOW_ETH_ALEN];
extern QueueHandle_t espnow_queue;

void app_espnow_init(void);

#ifdef DEBUG_LOGS
void print_hexadecimal_frame(const uint8_t *dest_mac, const uint8_t *payload, size_t payload_len);
#endif

#endif // ESP_NOW_TRANSMITE_H
