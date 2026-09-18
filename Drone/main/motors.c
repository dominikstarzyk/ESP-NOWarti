#include "motors.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "motors";

// LEDC configuration constants
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_14_BIT // 14-bit resolution (0 to 16383)
#define LEDC_FREQUENCY          (50)             // 50Hz frequency (20ms period)

// Function to map percentage (0-100) to duty cycle (1ms to 2ms pulse)
// At 50Hz, 20ms = 14-bit (16384 ticks)
// 1ms = (1/20) * 16384 = 819.2 ticks
// 2ms = (2/20) * 16384 = 1638.4 ticks
#define DUTY_MIN 819  // ~1ms
#define DUTY_MAX 1638 // ~2ms

typedef struct {
    int gpio;
    ledc_channel_t channel;
} motor_config_t;

static const motor_config_t motors[] = {
    { MOTOR_LF_GPIO, LEDC_CHANNEL_0 },
    { MOTOR_RF_GPIO, LEDC_CHANNEL_1 },
    { MOTOR_RR_GPIO, LEDC_CHANNEL_2 },
    { MOTOR_LR_GPIO, LEDC_CHANNEL_3 }
};

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static QueueHandle_t external_motor_queue = NULL;

static void motor_task(void *pvParameters)
{
    motor_speeds_t speeds;
    int log_divider = 0;
    
    while (1) {
        if (xQueueReceive(external_motor_queue, &speeds, portMAX_DELAY) == pdTRUE) {
            // Print received values once per second (approx every 50 messages at 20ms intervals)
            if (++log_divider >= 50) {
                ESP_LOGI(TAG, "Motor (1s Log) -> LF: %.2f, RF: %.2f, RR: %.2f, LR: %.2f", 
                         speeds.lf, speeds.rf, speeds.rr, speeds.lr);
                log_divider = 0;
            }

            float speed_array[4] = {speeds.lf, speeds.rf, speeds.rr, speeds.lr};

            for (int i = 0; i < 4; i++) {
                // Clamp input to 0-100%
                if (speed_array[i] < 0.0f) speed_array[i] = 0.0f;
                if (speed_array[i] > 100.0f) speed_array[i] = 100.0f;

                // Map 0-100 to DUTY_MIN - DUTY_MAX
                uint32_t duty = (uint32_t)(DUTY_MIN + (speed_array[i] / 100.0f) * (DUTY_MAX - DUTY_MIN));
                
                ledc_set_duty(LEDC_MODE, motors[i].channel, duty);
                ledc_update_duty(LEDC_MODE, motors[i].channel);
            }
        }
    }
}

esp_err_t motors_init(QueueHandle_t motor_queue)
{
    external_motor_queue = motor_queue;
    if (external_motor_queue == NULL) {
        ESP_LOGE(TAG, "Motor Initialization failed: Queue is NULL");
        return ESP_FAIL;
    }

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = LEDC_MODE,
            .channel        = motors[i].channel,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = motors[i].gpio,
            .duty           = DUTY_MIN, // Initialize at 0% speed (1ms)
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }

    // Start motor task
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Motors (PWM) initialized. Task listening to external queue.");
    return ESP_OK;
}
