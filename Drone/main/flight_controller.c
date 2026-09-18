#include "flight_controller.h"
#include "imu.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "flight_ctrl";

#define RAD_TO_DEG 57.2957795131f
#define FILTER_ALPHA 0.98f  // Complementary filter coefficient
#define DT 0.01f           // 100Hz = 10ms

static drone_attitude_t current_attitude = {0};
static QueueHandle_t global_imu_queue = NULL;

static void flight_controller_task(void *pvParameters)
{
    imu_msg_t imu_data;
    float roll = 0, pitch = 0;
    int log_divider = 0;

    while (1) {
        if (xQueueReceive(global_imu_queue, &imu_data, portMAX_DELAY) == pdTRUE) {
            
            // 1. Calculate angles from Accelerometer
            // Accel values are in m/s^2.
            float accel_roll = atan2f(imu_data.accel.y, imu_data.accel.z) * RAD_TO_DEG;
            float accel_pitch = atan2f(-imu_data.accel.x, sqrtf(imu_data.accel.y * imu_data.accel.y + imu_data.accel.z * imu_data.accel.z)) * RAD_TO_DEG;

            // 2. Complementary Filter
            // Combine Gyro integration with Accel absolute orientation
            // imu_data.gyro values are in deg/s.
            // Orientation: Gyro X=Roll rate, Gyro Y=Pitch rate
            roll = FILTER_ALPHA * (roll + imu_data.gyro.x * DT) + (1.0f - FILTER_ALPHA) * accel_roll;
            pitch = FILTER_ALPHA * (pitch + imu_data.gyro.y * DT) + (1.0f - FILTER_ALPHA) * accel_pitch;

            // 3. Update global state
            current_attitude.roll = roll;
            current_attitude.pitch = pitch;

            // 4. Periodic Log (1Hz)
            if (++log_divider >= 100) {
                ESP_LOGI(TAG, "Attitude -> Roll: %.2f | Pitch: %.2f", roll, pitch);
                log_divider = 0;
            }
        }
    }
}

esp_err_t flight_controller_init(QueueHandle_t imu_queue)
{
    global_imu_queue = imu_queue;
    if (global_imu_queue == NULL) {
        ESP_LOGE(TAG, "Failed to start: IMU queue is NULL");
        return ESP_FAIL;
    }

    xTaskCreate(flight_controller_task, "flight_ctrl_task", 4096, NULL, 15, NULL);
    
    ESP_LOGI(TAG, "Flight Controller Init: Complementary Filter active.");
    return ESP_OK;
}

void flight_controller_get_attitude(drone_attitude_t *attitude)
{
    if (attitude) {
        *attitude = current_attitude;
    }
}
