#include "esp_now_receive.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_idf_version.h"
#include "motors.h"
#include "imu.h"
#include "rgb_led.h"

static const char *TAG = "esp_now_rx";

#ifdef ENCRYPTION_MODE
// --- KLUCZE SZYFRUJĄCE CCMP (16 bajtów każdy) ---
// MUSZĄ BYĆ IDENTYCZNE Z KLUCZAMI W NADAJNIKU (PILOCIE)!
static const uint8_t pmk_key[16] = "MojeGlobalnePMK1"; 
static const uint8_t lmk_key[16] = "MojeSuperHaslo12"; 

// Adres MAC nadajnika (Pilota/Kontrolera)
static const uint8_t controller_mac[6] = { 0x10, 0x20, 0xBA, 0x42, 0x79, 0xB4 };
#endif

typedef struct {
    uint8_t mac_addr[6];
    uint8_t raw_data[sizeof(espnow_data_t)];
    int len;
} esp_now_event_t;

static QueueHandle_t esp_now_queue;
static QueueHandle_t global_motor_queue;
static bool is_calibrated = false;

static void esp_now_receive_task(void *pvParameter)
{
    esp_now_event_t evt;
    espnow_data_t latest_data = {0};
    int count = 0;
    TickType_t last_receive_tick = 0;
    bool connection_active = false;

    while (1) {
        bool received_new_data = false;
        
        while (xQueueReceive(esp_now_queue, &evt, 0) == pdTRUE) {
            // Bez względu na ENCRYPTION_MODE, pakiety w evt.raw_data są już odszyfrowane przez sprzęt!
            memcpy(&latest_data, evt.raw_data, sizeof(espnow_data_t));

            received_new_data = true;
            last_receive_tick = xTaskGetTickCount();
        }

        // --- Logika połączenia, kalibracji i sterowania silnikami ---
        if (received_new_data) {
            if (!connection_active) {
                connection_active = true;
                if (!is_calibrated) {
                    rgb_led_set_orange();
                    ESP_LOGI(TAG, "Connection established - LED Orange (Wait for calibration)");
                } else {
                    rgb_led_set_ready();
                }
            }

            if (latest_data.roll == 360.0f && latest_data.pitch == 360.0f && 
                latest_data.yaw == 360.0f && latest_data.throttle == 360.0f) {
                
                ESP_LOGI(TAG, "Calibration Triggered! Entering Calibration Mode...");
                rgb_led_set_blue();
                
                imu_calibrate();
                
                is_calibrated = true;
                rgb_led_set_ready();
                ESP_LOGI(TAG, "Calibration Complete - LED Green");
                
                latest_data.roll = 0; latest_data.pitch = 0; latest_data.yaw = 0; latest_data.throttle = 0;
            }
        } else {
            if (connection_active && (xTaskGetTickCount() - last_receive_tick > pdMS_TO_TICKS(500))) {
                connection_active = false;
                rgb_led_set_error();
                ESP_LOGW(TAG, "Connection lost - LED Red");
            }
        }
        
        if (++count >= 50) {
            if (connection_active) {
                ESP_LOGI(TAG, "[Telemetry] R:%.2f P:%.2f Y:%.2f T:%.2f | Calib:%d", 
                         latest_data.roll, latest_data.pitch, latest_data.yaw, latest_data.throttle, (int)is_calibrated);
            } else {
                ESP_LOGW(TAG, "Waiting for controller...");
            }
            count = 0;
        }
        
        motor_speeds_t motor_msg = {0};
        if (is_calibrated && connection_active) {
            motor_msg.lf = latest_data.throttle;
            motor_msg.rf = latest_data.throttle;
            motor_msg.rr = latest_data.throttle;
            motor_msg.lr = latest_data.throttle;
        }

        if (global_motor_queue != NULL) {
            xQueueSend(global_motor_queue, &motor_msg, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

#ifdef ENABLE_SNIFF
    #include "esp_wifi_types.h"

    // Funkcja drukująca dane 16-bitowo (4 bloki w linii)
    void print_raw_frame_16bit(const uint8_t *data, int len) {
        printf("\n=== SUROWA RAMKA RADIOWA 802.11 (%d bajtow) ===\n", len);
        int blocks_in_line = 0;
        
        for (int i = 0; i < len; i += 2) {
            if (i + 1 < len) {
                uint16_t value16 = (data[i] << 8) | data[i+1];
                printf("%04X ", value16);
            } else {
                printf("%02X00 ", data[i]);
            }
            
            blocks_in_line++;
            if (blocks_in_line == 4) {
                printf("\n");
                blocks_in_line = 0;
            }
        }
        if (blocks_in_line != 0) {
            printf("\n");
        }
        printf("================================================\n");
    }

    // Callback sniffera - przechwytuje pakiety bezpośrednio z radia
    static void wifi_sniffer_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
        if (type != WIFI_PKT_MGMT) return; 

        const wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
        const uint8_t *payload = pkt->payload;
        int len = pkt->rx_ctrl.sig_len;

        bool is_esp_now = false;
        for (int i = 0; i < len - 5; i++) {
            if (payload[i] == 0x18 && payload[i+1] == 0xFE && payload[i+2] == 0x34) {
                is_esp_now = true;
                break;
            }
        }

        if (is_esp_now) {
            print_raw_frame_16bit(payload, len);
        }
    }
#endif

// Callback wywoływany automatycznie po odebraniu pakietu ESP-NOW
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void esp_now_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len)
{
    const uint8_t *mac_addr = esp_now_info->src_addr;
#else
static void esp_now_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
#endif

#ifdef DEBUG_LOGS
    ESP_LOGI(TAG, "Odebrano pakiet od MAC: %02X:%02X:%02X:%02X:%02X:%02X (Dlugosc: %d bajtow)",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], len);
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, len, ESP_LOG_INFO);
#endif

    if (len == sizeof(espnow_data_t)) {
        esp_now_event_t evt;
        memcpy(evt.mac_addr, mac_addr, 6);
        memcpy(evt.raw_data, data, len); // Dane tutaj są już odszyfrowane przez sprzęt
        evt.len = len;

        if (esp_now_queue) {
            xQueueSendFromISR(esp_now_queue, &evt, NULL);
        }
    } else {
        ESP_LOGW(TAG, "Received unexpected data length: %d", len);
    }
}

// Inicjalizacja Wi-Fi oraz ESP-NOW
void esp_now_receive_init(QueueHandle_t motor_queue)
{
    global_motor_queue = motor_queue;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    uint8_t mac[6];
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
#else
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    esp_wifi_get_mac(WIFI_IF_STA, mac);
#endif
    ESP_LOGI(TAG, "My MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    esp_now_queue = xQueueCreate(10, sizeof(esp_now_event_t));
    if (esp_now_queue == NULL) {
        ESP_LOGE(TAG, "Error creating queue");
        return;
    }

    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing ESP-NOW");
        return;
    }

#ifdef ENCRYPTION_MODE
    // Ustawienie klucza PMK dla CCMP w trybie szyfrowanym
    ESP_ERROR_CHECK(esp_now_set_pmk(pmk_key));

    // W trybie szyfrowanym odbiornik musi zarejestrować nadajnik (Pilot) jako Peer z włączonym szyfrowaniem CCMP
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = true;                 // WŁĄCZAMY SZYFROWANIE CCMP
    memcpy(peer.lmk, lmk_key, 16);        // Przypisanie klucza LMK
    memcpy(peer.peer_addr, controller_mac, 6);

    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    ESP_LOGI(TAG, "ESP-NOW RX: Encrypted CCMP Peer (Controller) registered.");
#else
    ESP_LOGI(TAG, "ESP-NOW RX: Encryption DISABLED Mode.");
#endif

    if (esp_now_register_recv_cb(esp_now_recv_cb) != ESP_OK) {
        ESP_LOGE(TAG, "Error registering receive callback");
        return;
    }

    xTaskCreate(esp_now_receive_task, "esp_now_receive_task", 4096, NULL, 5, NULL);

#ifdef ENABLE_SNIFF
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
#endif

    ESP_LOGI(TAG, "ESP-NOW Receive Initialization Complete.");
}