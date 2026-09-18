#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_now_transmite.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

static const char *TAG = "app_esp_now";

#ifdef ENCRYPTION_MODE
// --- KLUCZE SZYFRUJĄCE CCMP (16 bajtów każdy) ---
// Zmienne używane TYLKO w trybie szyfrowanym
static const uint8_t pmk_key[16] = "MojeGlobalnePMK1"; 
static const uint8_t lmk_key[16] = "MojeSuperHaslo12"; 

// W trybie szyfrowanym podajemy konkretny MAC drona
uint8_t mac_receiver[ESP_NOW_ETH_ALEN] = { 0xB8, 0xF8, 0x62, 0xE2, 0xEB, 0x08 };
#else
// W trybie niezaszyfrowanym wysyłamy na Broadcast
uint8_t mac_receiver[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
#endif

QueueHandle_t espnow_queue = NULL;
static SemaphoreHandle_t espnow_tx_sem = NULL;

/* Callback wywoływany po wysłaniu pakietu w powietrze */
static void espnow_send_cb(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    if (info == NULL || info->des_addr == NULL) {
        return;
    }
    
    // Zwolnienie semafora, gdy pakiet opuścił bufor TX
    if (espnow_tx_sem != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(espnow_tx_sem, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static void espnow_task(void *pvParameter)
{
    espnow_data_t data;
    static uint32_t print_counter = 0;
    esp_err_t ret;

    while (1) {
        if (xQueueReceive(espnow_queue, &data, portMAX_DELAY) == pdTRUE) {
            // Logowanie co 25 pakietów (~2 razy na sekundę przy 50Hz)
            if (print_counter++ % 25 == 0) {
#ifdef ENCRYPTION_MODE
                ESP_LOGI(TAG, "Wireless Send (CCMP) -> R:%.2f P:%.2f Y:%.2f T:%.2f", 
                         data.roll, data.pitch, data.yaw, data.throttle);
#else
                ESP_LOGI(TAG, "Wireless Send (RAW) -> R:%.2f P:%.2f Y:%.2f T:%.2f", 
                         data.roll, data.pitch, data.yaw, data.throttle);
#endif
            }

            // Pobranie semafora z timeoutem 20ms dla kontroli przepływu (Flow Control)
            if (espnow_tx_sem != NULL && xSemaphoreTake(espnow_tx_sem, pdMS_TO_TICKS(20)) == pdTRUE) {

                // W obu przypadkach przesyłamy czystą strukturę!
                // Gdy ENCRYPTION_MODE jest włączone, radio ESP32 automatycznie zaszyfruje pakiet w sprzęcie (CCMP).
                ret = esp_now_send(mac_receiver, (uint8_t *)&data, sizeof(espnow_data_t));

                if (ret != ESP_OK) {
                    xSemaphoreGive(espnow_tx_sem); // Oddajemy semafor w przypadku natychmiastowego błędu
                    ESP_LOGE(TAG, "Send init error: %s", esp_err_to_name(ret));
                }
            } else {
                ESP_LOGD(TAG, "Buffer full (receiver offline) - frame dropped");
            }
        }
    }
}

void app_espnow_init(void)
{
    // 1. Tworzenie kolejki danych
    espnow_queue = xQueueCreate(10, sizeof(espnow_data_t));
    if (espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create espnow queue fail");
        return;
    }

    // 2. Tworzenie semafora nadawczego
    espnow_tx_sem = xSemaphoreCreateBinary();
    if (espnow_tx_sem != NULL) {
        xSemaphoreGive(espnow_tx_sem);
    } else {
        ESP_LOGE(TAG, "Create ESP-NOW TX Semaphore fail");
        return;
    }

    // 3. Inicjalizacja podstawowa ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

#ifdef ENCRYPTION_MODE
    // Ustawiamy PMK tylko w trybie szyfrowanym
    ESP_ERROR_CHECK(esp_now_set_pmk(pmk_key));
#endif

    // 4. Rejestracja Peera (odbiornika)
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    peer.channel = 0;               // Użyj bieżącego kanału STA
    peer.ifidx = WIFI_IF_STA;
    memcpy(peer.peer_addr, mac_receiver, ESP_NOW_ETH_ALEN);

#ifdef ENCRYPTION_MODE
    peer.encrypt = true;            // WŁĄCZAMY SZYFROWANIE SPRZĘTOWE CCMP
    memcpy(peer.lmk, lmk_key, 16);   // Przypisanie klucza LMK
#else
    peer.encrypt = false;           // BRAK SZYFROWANIA
#endif
    
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

#ifdef ENCRYPTION_MODE
    ESP_LOGI(TAG, "ESP-NOW Init Complete: CCMP Encryption ENABLED.");
#else
    ESP_LOGI(TAG, "ESP-NOW Init Complete: Encryption DISABLED (Broadcast mode).");
#endif

    // 5. Utworzenie taska nadawczego
    xTaskCreate(espnow_task, "espnow_task", 4096, NULL, 4, NULL);
}