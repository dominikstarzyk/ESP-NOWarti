#include "joystick.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_now_transmite.h"
#include "rgb_led.h"

static const char *TAG = "joystick";

#define JOYSTICK_ADC_CHAN_VRX ADC_CHANNEL_8 // GPIO9 on ESP32-S3 (Right X - Roll)
#define JOYSTICK_ADC_CHAN_VRY ADC_CHANNEL_9 // GPIO10 on ESP32-S3 (Right Y - Pitch)
#define JOYSTICK_SW_PIN GPIO_NUM_5          // Very common on S3 DevKits

#define JOYSTICK2_ADC_CHAN_VRX ADC_CHANNEL_0 // GPIO1 on ESP32-S3  (Left X - Yaw)
#define JOYSTICK2_ADC_CHAN_VRY ADC_CHANNEL_1 // GPIO2 on ESP32-S3 (Left Y - Throttle)
#define JOYSTICK2_SW_PIN GPIO_NUM_6          // Left Switch (Tweak me if another pin!)

// Calibration values (Adjust as needed if joystick is not perfectly centered)
#define VRX_MIN 0
#define VRX_MID 1996 // Calculated from 3.3V idle logs
#define VRX_MAX 4095

#define VRY_MIN 0
#define VRY_MID 1935 // Calculated from 3.3V idle logs
#define VRY_MAX 4095

// New Joystick Calibration Defaults
#define YAW_MIN 0
#define YAW_MID 1989 // Calculated from raw logs
#define YAW_MAX 4095

#define THR_MIN 0
#define THR_MID 2007 // Calculated from raw logs
#define THR_MAX 4095

// Scale limits (Degrees for R/P/Y, Percentage for Throttle)
#define ROLL_LIMIT    30.0f
#define PITCH_LIMIT   30.0f
#define YAW_LIMIT     60.0f
#define THROTTLE_LIMIT 100.0f


#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

static adc_oneshot_unit_handle_t adc1_handle;

void joystick_init(void)
{
    // Initialize switch pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << JOYSTICK_SW_PIN) | (1ULL << JOYSTICK2_SW_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Initialize ADC for ESP-IDF v5.0+
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = 0,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOYSTICK_ADC_CHAN_VRX, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOYSTICK_ADC_CHAN_VRY, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOYSTICK2_ADC_CHAN_VRX, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOYSTICK2_ADC_CHAN_VRY, &config));

    ESP_LOGI(TAG, "Joysticks initialized (VRX:CH%d, VRY:CH%d, YAW:CH%d, THR:CH%d, SW:GPIO%d) - ESP-IDF v5+",
             JOYSTICK_ADC_CHAN_VRX, JOYSTICK_ADC_CHAN_VRY, JOYSTICK2_ADC_CHAN_VRX, JOYSTICK2_ADC_CHAN_VRY, JOYSTICK_SW_PIN);
}

/**
 * @brief Optional filter to soften joystick response (Exponential/Cubic curve)
 * Keeps the center precision high and smooths out jumpy hardware.
 */
static float apply_joystick_filter(float value, float limit)
{
    if (limit == 0) return 0;
    float norm = value / limit;
    // Cubic curve: result = norm^3 * limit
    // This makes the center very soft but keeps full range at the ends.
    return (norm * norm * norm) * limit;
}

void joystick_read(float *roll, float *pitch, float *yaw, float *throttle, bool *sw1_pressed, bool *sw2_pressed)
{
    int vrx_raw = 0;
    int vry_raw = 0;
    int vrx2_raw = 0;
    int vry2_raw = 0;

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, JOYSTICK_ADC_CHAN_VRX, &vrx_raw));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, JOYSTICK_ADC_CHAN_VRY, &vry_raw));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, JOYSTICK2_ADC_CHAN_VRX, &vrx2_raw));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, JOYSTICK2_ADC_CHAN_VRY, &vry2_raw));


    if (sw1_pressed != NULL) {
        *sw1_pressed = (gpio_get_level(JOYSTICK_SW_PIN) == 0); // Active low with pull-up
    }
    if (sw2_pressed != NULL) {
        *sw2_pressed = (gpio_get_level(JOYSTICK2_SW_PIN) == 0); // Active low with pull-up
    }

#ifdef DEBUG_LOGSV2
    static uint32_t debug_counter = 0;
    if (debug_counter++ % 25 == 0) {
        ESP_LOGI(TAG, "RAW ADC: L_VRX(Yaw):%d L_VRY(Throttle):%d R_VRX(Roll):%d R_VRY(Pitch):%d SWLeft:%d SWRight:%d", 
                 vrx2_raw, vry2_raw, vrx_raw, vry_raw, *sw1_pressed, *sw2_pressed);
    }
#endif

    if (roll != NULL) {
        if (vrx_raw < VRX_MID) {
            *roll = ((float)(vrx_raw - VRX_MID) / (VRX_MID - VRX_MIN)) * ROLL_LIMIT;
        } else {
            *roll = ((float)(vrx_raw - VRX_MID) / (VRX_MAX - VRX_MID)) * ROLL_LIMIT;
        }
        if (*roll < -ROLL_LIMIT) *roll = -ROLL_LIMIT;
        if (*roll > ROLL_LIMIT) *roll = ROLL_LIMIT;
        if (*roll > -0.4f && *roll < 0.4f) *roll = 0.0f; // Deadband

        // --- FILTER (Comment out to detach) ---
        *roll = apply_joystick_filter(*roll, ROLL_LIMIT);
    }
    
    if (pitch != NULL) {
        if (vry_raw < VRY_MID) {
            *pitch = ((float)(vry_raw - VRY_MID) / (VRY_MID - VRY_MIN)) * PITCH_LIMIT;
        } else {
            *pitch = ((float)(vry_raw - VRY_MID) / (VRY_MAX - VRY_MID)) * PITCH_LIMIT;
        }
        if (*pitch < -PITCH_LIMIT) *pitch = -PITCH_LIMIT;
        if (*pitch > PITCH_LIMIT) *pitch = PITCH_LIMIT;
        if (*pitch > -0.4f && *pitch < 0.4f) *pitch = 0.0f; // Deadband

        // --- FILTER (Comment out to detach) ---
        *pitch = apply_joystick_filter(*pitch, PITCH_LIMIT);
    }

    if (yaw != NULL) {
        if (vrx2_raw < YAW_MID) {
            *yaw = ((float)(vrx2_raw - YAW_MID) / (YAW_MID - YAW_MIN)) * YAW_LIMIT;
        } else {
            *yaw = ((float)(vrx2_raw - YAW_MID) / (YAW_MAX - YAW_MID)) * YAW_LIMIT;
        }
        if (*yaw < -YAW_LIMIT) *yaw = -YAW_LIMIT;
        if (*yaw > YAW_LIMIT) *yaw = YAW_LIMIT;
        if (*yaw > -0.4f && *yaw < 0.4f) *yaw = 0.0f; // Deadband

        // --- FILTER (Comment out to detach) ---
        *yaw = apply_joystick_filter(*yaw, YAW_LIMIT);
    }
    
    if (throttle != NULL) {
        if (vry2_raw < THR_MID) {
            *throttle = ((float)(vry2_raw - THR_MID) / (THR_MID - THR_MIN)) * THROTTLE_LIMIT;
        } else {
            *throttle = ((float)(vry2_raw - THR_MID) / (THR_MAX - THR_MID)) * THROTTLE_LIMIT;
        }
        if (*throttle < -THROTTLE_LIMIT) *throttle = -THROTTLE_LIMIT;
        if (*throttle > THROTTLE_LIMIT) *throttle = THROTTLE_LIMIT;
        if (*throttle > -0.4f && *throttle < 0.4f) *throttle = 0.0f; // Deadband

        // --- FILTER (Comment out to detach) ---
        *throttle = apply_joystick_filter(*throttle, THROTTLE_LIMIT);
    }
}


void joystick_task(void *pvParameters)
{
    espnow_data_t drone_data;
    float joystick_roll = 0.0f;
    float joystick_pitch = 0.0f;
    float joystick_yaw = 0.0f;
    float joystick_throttle = 0.0f;
    bool sw1_pressed = false;
    bool sw2_pressed = false;

    // Calibration state machine
    typedef enum {
        MODE_NORMAL,
        MODE_CALIB_ZERO_INIT,
        MODE_CALIB_360,
        MODE_CALIB_ZERO_WAIT
    } calib_mode_t;
    
    calib_mode_t current_mode = MODE_NORMAL;
    int calib_packet_count = 0;
    bool sw_trigger_prev = false;

    ESP_LOGI(TAG, "Joystick task started.");
    uint32_t loop_counter = 0;

    while (1) {
        joystick_read(&joystick_roll, &joystick_pitch, &joystick_yaw, &joystick_throttle, &sw1_pressed, &sw2_pressed);

        bool sw_trigger = sw1_pressed && sw2_pressed;
        bool sw_new_press = sw_trigger && !sw_trigger_prev;
        sw_trigger_prev = sw_trigger;

        switch (current_mode) {
            case MODE_NORMAL:
                drone_data.roll = joystick_roll;
                drone_data.pitch = joystick_pitch;
                drone_data.yaw = joystick_yaw;
                drone_data.throttle = joystick_throttle;
                
                if (sw_new_press) {
                    current_mode = MODE_CALIB_ZERO_INIT;
                    rgb_led_set_blue(); // Calibration mode -> BLUE
                    ESP_LOGI(TAG, "Calibration Triggered - Step 1: Initial Zero");
                }
                break;

            case MODE_CALIB_ZERO_INIT:
                drone_data.roll = 0.0f;
                drone_data.pitch = 0.0f;
                drone_data.yaw = 0.0f;
                drone_data.throttle = 0.0f;
                
                current_mode = MODE_CALIB_360;
                calib_packet_count = 0;
                break;

            case MODE_CALIB_360:
                drone_data.roll = 360.0f;
                drone_data.pitch = 360.0f;
                drone_data.yaw = 360.0f;
                drone_data.throttle = 360.0f;
                
                calib_packet_count++;
                if (calib_packet_count >= 5) {
                    current_mode = MODE_CALIB_ZERO_WAIT;
                    ESP_LOGI(TAG, "Calibration Mode - Step 2: Sending 360s done. Waiting for exit trigger.");
                }
                break;

            case MODE_CALIB_ZERO_WAIT:
                drone_data.roll = 0.0f;
                drone_data.pitch = 0.0f;
                drone_data.yaw = 0.0f;
                drone_data.throttle = 0.0f;
                
                if (sw_new_press) {
                    current_mode = MODE_NORMAL;
                    rgb_led_set_ready(); // Normal mode -> GREEN
                    ESP_LOGI(TAG, "Calibration Mode - Exiting to Normal Operation.");
                }
                break;
        }

        // Print logs
        if (loop_counter % 25 == 0) {
            if (current_mode == MODE_NORMAL) {
                ESP_LOGI(TAG, "Sending Joystick Values - R:%.2f P:%.2f Y:%.2f T:%.2f SW1:%d SW2:%d", 
                         drone_data.roll, drone_data.pitch, drone_data.yaw, drone_data.throttle, sw1_pressed, sw2_pressed);
            } else {
                ESP_LOGI(TAG, "CALIBRATION MODE [%d] - R:%.2f P:%.2f Y:%.2f T:%.2f", 
                         current_mode, drone_data.roll, drone_data.pitch, drone_data.yaw, drone_data.throttle);
            }
        }
        loop_counter++;

        if (espnow_queue != NULL) {
            xQueueSend(espnow_queue, &drone_data, portMAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Update at 50Hz for low latency control
    }
}
